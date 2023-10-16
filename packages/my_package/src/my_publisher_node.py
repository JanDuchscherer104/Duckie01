#!/usr/bin/env python3

import os
from typing import Any

import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String


class MyPublisherNode(DTROS):
    _PUB_RATE = 1

    def __init__(self, node_name: str) -> None:
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC
        )

        # static parameters
        self._vehicle_name = os.environ["VEHICLE_NAME"]

        # construct publisher
        self._publisher = rospy.Publisher(
            f"/{self._vehicle_name}/my_publisher_node/message", String, queue_size=10
        )

        # set up a timer to call the publishing method
        self._timer = rospy.Timer(
            rospy.Duration.from_sec(self._PUB_RATE), self.pub_message
        )

        rospy.loginfo(f"Publisher {node_name} has started quacking!")

    def pub_message(self, event: Any) -> None:
        message = f"Quack quack from {self._vehicle_name}!"
        rospy.loginfo(f"Publishing message: '{message}'")
        self._publisher.publish(message)


if __name__ == "__main__":
    # create the node
    node = MyPublisherNode(node_name="my_publisher_node")
    # keep the process from terminating
    rospy.spin()
