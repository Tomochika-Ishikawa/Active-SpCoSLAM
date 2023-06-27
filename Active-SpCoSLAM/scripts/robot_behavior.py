#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from init import *

def save_map(step, data_path):
    rospy.wait_for_message("/map", nav_msgs.OccupancyGrid)
    save_path = data_path + "/maps/" + str(step)
    subprocess.Popen("rosrun map_server map_saver map:=/map -f " + save_path, shell=True)

def save_image(step, angle_index, image_data, data_path):
    save_path = data_path + "/images/" + str(step) + "_" + str(angle_index) + ".png"
    cv2.imwrite(save_path, image_data)

def turn_around(twist_pub):
    rotate_angle_num = 3
    robot_speed = 0.4
    rotate_time = (math.pi/2) / robot_speed
    twist = geometry_msgs.Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = robot_speed

    start_time = time.time()
    pass_time = 0
    while pass_time < rotate_time:
        twist_pub.publish(twist)
        pass_time = time.time() - start_time

def move_to_destination(x, y, z, step, move_base_act):
    start_all_step_moving_time = time.time()

    alpha = (180 / math.pi) * z
    move_base_act.wait_for_server()
    goal_pose = geometry_msgs.Pose()
    goal_pose.position.x = x
    goal_pose.position.y = y
    goal_pose.position.z = 0.0
    goal_pose.orientation = Quaternion_from_euler(alpha)

    goal = geometry_msgs.PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    goal.pose = goal_pose
    send_goal = move_base_msgs.MoveBaseGoal()
    send_goal.target_pose = goal

    move_base_act.send_goal(send_goal)
    rospy.loginfo('[Active-SpCoSLAM]         Move to (%.2f, %.2f)', goal_pose.position.x, goal_pose.position.y)

    move_base_act.wait_for_result(rospy.Duration(120))
    if move_base_act.get_state() == actionlib_msgs.GoalStatus.SUCCEEDED:
        rospy.loginfo('[Active-SpCoSLAM]         Succeeded')
        move_state = True
    else:
        rospy.loginfo('[Active-SpCoSLAM]         Failure')
        move_state = False

    move_base_act.cancel_all_goals()  

    #Reset head
    # hsr = hsrb_interface.Robot()
    # whole_body = hsr.get('whole_body')
    # whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': 0.0})

    if step != -1:
        moving_time = time.time() - start_all_step_moving_time
    return move_state, moving_time