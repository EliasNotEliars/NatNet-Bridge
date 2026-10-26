#!/usr/bin/env python3
"""ROS 1 bridge for the NatNet 4.5.1 Python client (compatible with 4.1.0)

Run after building and sourcing the catkin workspace:
    roslaunch natnet_bridge natnet.launch

Private ROS parameters:
    ~client_address    Debian interface address
    ~server_address    Motive server address
    ~use_multicast     Whether to use multicast
    ~frame_id          Coordinate-frame label for published messages
    ~rigid_body_names  Mapping of Streaming IDs to readable ROS names

Coordinates pass through unchanged. Timestamps are local receipt times.
This node does not publish TF; optitrack_tf.py is responsible for that.
"""

import re
import socket
import threading
import rospy
from geometry_msgs.msg import Point32, PoseStamped
from sensor_msgs.msg import PointCloud

from natnet_bridge.NatNetClient import NatNetClient


class NatNetNode:
    def __init__(self):
        rospy.init_node("natnet_node")

        # Read configuration before starting the NatNet receiver threads.
        self.frame_id = rospy.get_param(
            "~frame_id", "optitrack_world"
        )
        client_address = rospy.get_param(
            "~client_address", "10.0.0.1"
        )
        server_address = rospy.get_param(
            "~server_address", "10.0.0.2"
        )
        use_multicast = rospy.get_param(
            "~use_multicast", True
        )

        if not isinstance(use_multicast, bool):
            raise ValueError("~use_multicast must be a boolean.")

        self.rigid_body_names = self.read_body_names()

        # Publishers are created once when each body or marker set appears.
        self.pose_publishers = {}
        self.marker_publishers = {}

        # Serialize callbacks if frames arrive through both SDK sockets.
        self.callback_lock = threading.Lock()
        self.stopping = threading.Event()

        self.streaming_client = NatNetClient()
        self.streaming_client.set_client_address(client_address)
        self.streaming_client.set_server_address(server_address)
        self.streaming_client.set_use_multicast(use_multicast)
        self.streaming_client.set_print_level(0)
        self.streaming_client.new_frame_with_data_listener = (
            self.publish_frame
        )

    def read_body_names(self):
        """Read and validate the Streaming-ID-to-topic-name mapping."""
        raw_names = rospy.get_param("~rigid_body_names", {})

        if not isinstance(raw_names, dict):
            raise ValueError(
                "~rigid_body_names must be a dictionary "
                'such as {"1": "cs200", "2": "tello"}.'
            )

        body_names = {}

        for raw_id, name in raw_names.items():
            # YAML keys should be quoted integer strings.
            if re.fullmatch(r"[0-9]+", str(raw_id)) is None:
                raise ValueError(
                    f"Invalid rigid body ID: {raw_id!r}"
                )

            body_id = int(raw_id)

            if body_id in body_names:
                raise ValueError(
                    f"Duplicate rigid body ID: {body_id}"
                )

            if (
                not isinstance(name, str)
                or re.fullmatch(
                    r"[A-Za-z][A-Za-z0-9_]*", name
                ) is None
            ):
                raise ValueError(
                    f"Invalid name for body {body_id}: {name!r}. "
                    "Use a letter followed by letters, "
                    "digits or underscores."
                )

            # Prevent aliases from colliding with unnamed-body topics.
            if re.fullmatch(r"id_[0-9]+", name):
                raise ValueError(
                    f"{name!r} is reserved for unnamed bodies."
                )

            if name in body_names.values():
                raise ValueError(
                    f"Duplicate rigid body name: {name!r}"
                )

            body_names[body_id] = name

        return body_names

    def start(self):
        if not self.streaming_client.run():
            raise RuntimeError("Could not start NatNet client.")
            

        rospy.loginfo(
            "NatNet receivers started; waiting for frames."
        )

    def pose_publisher(self, body_id):
        """Return the existing publisher, or create it on first sight."""
        if body_id not in self.pose_publishers:
            body_name = self.rigid_body_names.get(
                body_id, f"id_{body_id}"
            )

            topic = f"/optitrack/rigid_bodies/{body_name}/pose"

            self.pose_publishers[body_id] = rospy.Publisher(topic, PoseStamped, queue_size=1)

            rospy.loginfo(
                "Discovered rigid body %d -> %s",
                body_id,
                topic,
            )

        return self.pose_publishers[body_id]

    def marker_publisher(self, raw_name):
        """Create a cloud publisher for each named marker set."""
        name = (
            raw_name
            if isinstance(raw_name, bytes)
            else raw_name.encode("utf-8")
        )

        if name not in self.marker_publishers:
            # Hex encoding avoids invalid characters and name collisions.
            topic = (
                f"/optitrack/marker_sets/name_{name.hex()}/markers"
            )

            self.marker_publishers[name] = rospy.Publisher(
                topic,
                PointCloud,
                queue_size=1,
            )

            rospy.loginfo(
                "Discovered marker set %r -> %s",
                name,
                topic,
            )

        return name, self.marker_publishers[name]

    def cloud(self, positions, stamp):
        message = PointCloud()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id

        message.points = [
            Point32(x=p[0], y=p[1], z=p[2])
            for p in positions
        ]

        return message

    def publish_frame(self, data_dict):
        with self.callback_lock:
            if self.stopping.is_set() or rospy.is_shutdown():
                return

            data = data_dict.get("mocap_data")
            if data is None:
                return

            stamp = rospy.Time.now()

            # Publish the pose of each valid rigid body.
            if data.rigid_body_data is not None:
                for body in data.rigid_body_data.rigid_body_list:
                    publisher = self.pose_publisher(body.id_num)

                    if not body.tracking_valid:
                        continue

                    message = PoseStamped()
                    message.header.stamp = stamp
                    message.header.frame_id = self.frame_id

                    message.pose.position.x = body.pos[0]
                    message.pose.position.y = body.pos[1]
                    message.pose.position.z = body.pos[2]

                    message.pose.orientation.x = body.rot[0]
                    message.pose.orientation.y = body.rot[1]
                    message.pose.orientation.z = body.rot[2]
                    message.pose.orientation.w = body.rot[3]

                    publisher.publish(message)

            # Publish individual points for each named marker set.
            # These sets may contain model-filled positions.
            seen_sets = set()

            if data.marker_set_data is not None:
                for markers in data.marker_set_data.marker_data_list:
                    # "all" duplicates the markers from the other sets.
                    if markers.model_name in (b"all", "all"):
                        continue

                    name, publisher = self.marker_publisher(
                        markers.model_name
                    )
                    seen_sets.add(name)

                    publisher.publish(
                        self.cloud(markers.marker_pos_list, stamp)
                    )

            # Clear clouds for sets absent from this frame.
            for name, publisher in self.marker_publishers.items():
                if name not in seen_sets:
                    publisher.publish(self.cloud([], stamp))

    def shutdown(self):
        if self.stopping.is_set():
            return

        self.stopping.set()
        client = self.streaming_client
        client.stop_threads = True

        # Wake blocked UDP receivers before closing their sockets.
        for sock in (client.command_socket, client.data_socket):
            if sock is not None:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass

                sock.close()

        for thread in (client.command_thread, client.data_thread):
            if (
                thread is not None
                and thread is not threading.current_thread()
                and thread.is_alive()
            ):
                thread.join(timeout=3.0)

                if thread.is_alive():
                    rospy.logwarn(
                        "NatNet receiver did not stop: %s",
                        thread.name,
                    )


def main():
    node = None

    try:
        node = NatNetNode()
        node.start()
        rospy.spin()

    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass

    except Exception as error:
        rospy.logerr("NatNet node failed: %s", error)
        return 1

    finally:
        if node is not None:
            node.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())