#!/usr/bin/env python

import sys
import geometry_msgs.msg
import moveit_commander
import rospy
improt moveit_msgs.msg import RobotState
from my_robot_arm_service.srv import MoverService, MoverServiceResponse
from sensor_msgs.msg import JointState

def receive_request(req):
    move_group = moveit_commander.MoveGroupCommander('my_robot_arm')
    plan = plan_trajectory(
        move_group, req.joints_input.goal_pose, req.joints_input.joints
    )
    move_group.stop()
    move_group.clear_pose_target()

    response = MoverServiceResponse()
    if plan.joint_trajectory.points:
        print('success')
        response.trajectory = plan
    else:
        print('fail') 
    return response

def plan_trajectory(move_group, pose_target, start_joints):
    joint_state = JointState()
    joint_state.name = ['hip','shoulder','elbow','wrist']
    joint_state.position = start_joints
    robot_state = RobotState()
    robot_state.joint_state = joint_state
    move_group.set_start_state(robot_state)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position = pose_target.position
    pose_goal.orientation = pose_target.orientation
    move_group.set_joint_value_target(pose_goal, True)

    return move_group.plan()[1]

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('mover')
    rospy.Service('my_robot_arm_server', MoverService, receive_request)
    print('Ready to plan')
    rospy.spin()

if __name__=="__main__":
    main()







