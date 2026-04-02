#!/usr/bin/python3
import rospy
import tf2_ros
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import TransformStamped

class FiducialTFBroadcaster:
    def __init__(self):
        self.parent_frame = rospy.get_param("~parent_frame", "camera_link")
        self.br = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/fiducial_transforms",
                         FiducialTransformArray,
                         self.cb)

    def cb(self, msg):
        for ft in msg.transforms:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.parent_frame
            t.child_frame_id = f"fiducial_{ft.fiducial_id}"

            t.transform.translation.x = ft.transform.translation.x
            t.transform.translation.y = ft.transform.translation.y
            t.transform.translation.z = ft.transform.translation.z

            t.transform.rotation = ft.transform.rotation
            self.br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("fiducial_tf_broadcaster")
    FiducialTFBroadcaster()
    rospy.spin()
