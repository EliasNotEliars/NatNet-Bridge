#!/usr/bin/env python3
"""Connect NatNet PoseStamped output to the drone's URDF TF tree.

world -> optitrack_world: static room alignment
optitrack_world -> tello_mocap: measured rigid-body pose
tello_mocap -> tello_base: static mounting calibration
tello_base -> drone links: robot_state_publisher
"""

import math
import time
import rospy
import tf2_ros
from jsk_rviz_plugins.msg import OverlayText
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker


def normalized_quaternion(values):
    if len(values) != 4 or not all(math.isfinite(v) for v in values):
        raise ValueError("Quaternion must contain four finite values: x, y, z, w")
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 1e-8:
        raise ValueError("Quaternion has zero length")
    return [v / norm for v in values]


def make_transform(parent, child, translation, quaternion, stamp):
    if len(translation) != 3 or not all(math.isfinite(v) for v in translation):
        raise ValueError("Translation must contain three finite values")
    q = normalized_quaternion(quaternion)
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = translation[0]
    msg.transform.translation.y = translation[1]
    msg.transform.translation.z = translation[2]
    msg.transform.rotation.x = q[0]
    msg.transform.rotation.y = q[1]
    msg.transform.rotation.z = q[2]
    msg.transform.rotation.w = q[3]
    return msg


class OptiTrackTF:
    def __init__(self):
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.source_frame = rospy.get_param("~optitrack_frame", "optitrack_world")
        self.marker_frame = rospy.get_param("~marker_frame", "tello_mocap")
        self.base_frame = rospy.get_param("~base_frame", "tello_base")
        frames = [self.world_frame, self.source_frame, self.marker_frame, self.base_frame]
        if len(set(frames)) != 4 or any(not f or f.startswith("/") for f in frames):
            raise ValueError("Frame IDs must be distinct, nonempty and have no leading slash")

        self.tracking_missing = False

        # Reuse natnet_node's ID-to-name mapping; no duplicate alias config.
        body_id = int(rospy.get_param("~rigid_body_id", 1))
        names = rospy.get_param("~rigid_body_names", {})
        names = {int(key): value for key, value in names.items()}
        body_name = names.get(body_id, "id_%d" % body_id)
        self.pose_topic = "/optitrack/rigid_bodies/%s/pose" % body_name
        self.timeout = float(rospy.get_param("~pose_timeout", 1.0))
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("pose_timeout must be positive and finite")

        # Values describe the child frame's origin and orientation in its parent.
        world_alignment = rospy.get_param("~world_to_optitrack")
        mounting = rospy.get_param("~mocap_to_base")
        stamp = rospy.Time.now()
        static_transforms = [
            make_transform(self.world_frame, self.source_frame,
                           world_alignment["translation"],
                           world_alignment["rotation_xyzw"], stamp),
            make_transform(self.marker_frame, self.base_frame,
                           mounting["translation"],
                           mounting["rotation_xyzw"], stamp),
        ]
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.broadcaster = tf2_ros.TransformBroadcaster()
        # Publish both together so late subscribers receive both static edges.
        self.static_broadcaster.sendTransform(static_transforms)

        self.last_received = None
        self.started = time.monotonic()
        self.body_name = body_name
        # Anchor warnings in world, so they do not require a tracked drone TF.
        self.status_position = rospy.get_param("~status_position", [0.0, 0.0, 1.5])
        if (len(self.status_position) != 3
                or not all(math.isfinite(v) for v in self.status_position)):
            raise ValueError("status_position must contain three finite values")
        self.status_publisher = rospy.Publisher(
            "~tracking_status", Marker, queue_size=1, latch=True
        )
        self.subscriber = rospy.Subscriber(
            self.pose_topic, PoseStamped, self.pose_callback, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(0.2), self.check_tracking)
        rospy.loginfo("OptiTrack TF: subscribing to %s", self.pose_topic)

    def pose_callback(self, pose):
        if rospy.is_shutdown():
            return
        if pose.header.frame_id != self.source_frame:
            rospy.logwarn_throttle(
                5.0, "Ignoring pose in frame '%s'; expected '%s'.",
                pose.header.frame_id, self.source_frame
            )
            return
        if pose.header.stamp == rospy.Time(0):
            rospy.logwarn_throttle(5.0, "Ignoring pose with a zero timestamp")
            return
        p, q = pose.pose.position, pose.pose.orientation
        try:
            transform = make_transform(
                self.source_frame, self.marker_frame,
                [p.x, p.y, p.z], [q.x, q.y, q.z, q.w], pose.header.stamp
            )
        except ValueError as error:
            rospy.logwarn_throttle(5.0, "Ignoring invalid pose: %s", error)
            return
        self.broadcaster.sendTransform(transform)
        self.last_received = time.monotonic()

    def check_tracking(self, _event):
        last_received = self.last_received
        reference = self.started if last_received is None else last_received
        age = time.monotonic() - reference
        missing = age > self.timeout

        status = Marker()
        status.header.frame_id = self.world_frame
        status.header.stamp = rospy.Time(0)
        status.ns = "optitrack_tracking_status"
        status.id = 0
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD if missing else Marker.DELETE
        status.pose.orientation.w = 1.0
        status.pose.position.x = self.status_position[0]
        status.pose.position.y = self.status_position[1]
        status.pose.position.z = self.status_position[2]
        status.scale.z = 0.12  # Text height in metres.
        status.color.r = 1.0
        status.color.g = 0.25
        status.color.b = 0.05
        status.color.a = 1.0
        # Refresh while missing; expire if this node itself stops.
        status.lifetime = rospy.Duration(1.0)
        status.frame_locked = True
        if missing:
            state = "No valid pose received" if last_received is None else "Last valid pose %.1f s ago" % age
            status.text = (
                "%s: TRACKING UNAVAILABLE\n%s\n"
                "Check marker visibility and Motive streaming"
            ) % (self.body_name, state)
        if missing and not self.tracking_missing:
            rospy.logwarn(
                "No usable OptiTrack pose found on %s.\n"
                "The drone TF is not being refreshed.",
                self.pose_topic
            )
        elif not missing and self.tracking_missing:
            rospy.loginfo(
                "OptiTrack tracking restored on %s. Drone TF is updating again.",
                self.pose_topic
            )

        self.tracking_missing = missing
        self.status_publisher.publish(status)


def main():
    rospy.init_node("optitrack_tf")
    try:
        node = OptiTrackTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as error:
        rospy.logfatal("OptiTrack TF failed: %s", error)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
