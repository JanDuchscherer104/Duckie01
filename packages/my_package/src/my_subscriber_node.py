#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

SUB_TOPIC = "chatter"


class MySubscriberNode(DTROS):
    def __init__(self, node_name: str) -> None:
        """
        Initializes the DTROS parent class and sets up a subscriber.

        Args:
            node_name (str): The name of the node.
        """
        # Initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC
        )

        # Initialize the subscriber and set the callback function
        self.sub = rospy.Subscriber(SUB_TOPIC, String, self.sub_callback)

    def sub_callback(self, data: String) -> None:
        """
        Callback function for the subscriber.

        Args:
            data (String): The data received from the publisher.
        """
        rospy.loginfo("I heard '%s'", data.data)


if __name__ == "__main__":
    # Create the node
    node = MySubscriberNode(node_name="my_subscriber_node")

    # Keep spinning to keep the program alive
    rospy.spin()
