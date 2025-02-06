#!/usr/bin/env python3  
import rospy
import tf
   

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((9.7, 2.5, 1.2),
                     tf.transformations.quaternion_from_euler(0, 0, 3.14143),
                         rospy.Time.now(),
                         "tag_0",
                         "map")
        br.sendTransform((0, -1.5, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "tag_1",
                         "tag_0")
        br.sendTransform((8.40, 6.7, 1.2),
                     tf.transformations.quaternion_from_euler(0,0,-1.57068),
                         rospy.Time.now(),
                         "tag_3",
                         "map")
        br.sendTransform((0, -1.5, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "tag_4",
                         "tag_3")
        br.sendTransform((0, -3.0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "tag_5",
                         "tag_3")
        br.sendTransform((0, -4.5, 0),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "tag_6",
                         "tag_3")
        rate.sleep()