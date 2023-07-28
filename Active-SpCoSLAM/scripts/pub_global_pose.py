#!/usr/bin/env python
import rospy
import subprocess
import tf2_ros
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('publish_global_pose')
    p = subprocess.Popen("rosnode kill /pose_integrator", shell=True)
    tf2_buffer = tf2_ros.Buffer()
    tf2_listener = tf2_ros.TransformListener(tf2_buffer)
    pose_pub = rospy.Publisher('/global_pose', PoseStamped, queue_size=10)
    # rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        try:
            map_to_robot = tf2_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(0.1))
            # map_to_robot = tf2_buffer.lookup_transform('map', 'base_footprint', rospy.Time.now())
            msg = PoseStamped()
            msg.header.stamp = map_to_robot.header.stamp
            msg.header.frame_id = 'map'
            msg.pose.position = map_to_robot.transform.translation
            msg.pose.orientation = map_to_robot.transform.rotation
            pose_pub.publish(msg)
        except Exception:
            pass
        # rate.sleep()

# #!/usr/bin/env python3
# import rospy
# import subprocess
# import tf2_ros
# from geometry_msgs.msg import PoseStamped

# if __name__ == '__main__':
#     rospy.loginfo("Publish robot pose topic")
#     rospy.init_node('tf_to_global_pose')
#     p = subprocess.Popen("rosnode kill /pose_integrator", shell=True)
#     tf2_buffer = tf2_ros.Buffer()
#     tf2_listener = tf2_ros.TransformListener(tf2_buffer)
#     pose_pub = rospy.Publisher('/global_pose', PoseStamped, queue_size=1)

#     while not rospy.is_shutdown():
#         time_now = rospy.Time.now()
#         while not rospy.is_shutdown():
#             try:
#                 print('try')
#                 #TFはGmappingから更新されるので、これでGmappingから自己位置推定結果を取ってきたことになる
#                 #実機だと計算が重くて動かない
#                 print(0)
#                 # map_to_robot = tf2_buffer.lookup_transform('map','base_footprint',  time_now-0.1)#, rospy.Duration(1))
#                 map_to_robot = tf2_buffer.lookup_transform('map','base_footprint',  rospy.Time.now())
#                 print(1)
#                 msg = PoseStamped()
#                 msg.header.stamp = map_to_robot.header.stamp
#                 msg.header.frame_id = 'map'
#                 msg.pose.position = map_to_robot.transform.translation
#                 msg.pose.orientation = map_to_robot.transform.rotation
#                 pose_pub.publish(msg)
                
#             except Exception:
#                 print("pass")
#                 # rospy.sleep(0.01)
#                 pass
