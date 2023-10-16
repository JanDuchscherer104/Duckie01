#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

SUB_TOPIC = "chatter"


class MySubscriberNode(DTROS):
    def __init__(self, node_name: str) -> None:
        # Initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC
        )

        # Initialize the subscriber and set the callback function
        self.sub = rospy.Subscriber(SUB_TOPIC, String, self.sub_callback)

        rospy.loginfo(
            f"Subscriber {node_name} has been started! Now listening on {SUB_TOPIC}"
        )

    def sub_callback(self, data: String) -> None:
        rospy.loginfo(f"I heard '{data.data}'!")


if __name__ == "__main__":
    # Create the node
    node = MySubscriberNode(node_name="my_subscriber_node")

    # Keep spinning to keep the program alive
    rospy.spin()
