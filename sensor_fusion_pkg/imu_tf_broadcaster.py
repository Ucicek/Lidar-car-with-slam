#!/usr/bin/env python
import rospy
import tf

def main():
    rospy.init_node('imu_tf_broadcaster')

    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0, 0, 0),
                 tf.transformations.quaternion_from_euler(0, 0, 0),
                 rospy.Time.now(),
                 "plane",
                 "base_footprint")
	#br.sendTransform((0, 0, 0),
         #        tf.transformations.quaternion_from_euler(0, 0, 0),
          #       rospy.Time.now(),
           #      "plane",
            #     "base_footprint")

        rate.sleep()

if __name__ == '__main__':
    main()

