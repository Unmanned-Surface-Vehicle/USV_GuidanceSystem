#!/usr/bin/env python
# license removed for brevity

import  rospy
from    std_msgs.msg import String
# import  sensor_msgs
from    sensor_msgs.msg import JointState

def airboat_guider():

    # ROS node initialization
    rospy.init_node('airboat_guider', anonymous=True)

    # Publishers for joint turning and thruster control
    pub_joint               = rospy.Publisher('/airboat/joint_states',      JointState, queue_size=10)
    pub_thruster            = rospy.Publisher('/airboat/thruster_command',  JointState, queue_size=10)

    # Publish freqeuncy for both msgs (joint and thruster)
    rate                    = rospy.Rate(1000) # 10hz

    # Handle for publishing
    joint_state_msg         = JointState()
    thruster_command_msg    = JointState()

    # Values to be published
    joint_position_vector       = [1]
    thruster_position_vector    = [4]

    while not rospy.is_shutdown():

        joint_state_msg.position        = ['fwd_joint']
        joint_state_msg.position        = joint_position_vector

        thruster_command_msg.name       = ['fwd']
        thruster_command_msg.position   = thruster_position_vector
        

        pub_joint.publish(joint_state_msg)
        pub_thruster.publish(thruster_command_msg)

        rate.sleep()

def JSHOP_to_ROS():
    # open JSHOP plan ".txt" file
    JSHOP_Plan = open("/home/darlan/Darlan/Projects/Artificial-Inteligence/Planning/Hierarchical-Task-Network/jshop2/examples/colregs/plan.txt","r") #opens file with name of "test.txt"

    # test if planner identified a possible collision based on current state
    # change action
    first_line = JSHOP_Plan.readline()                      
    if "!collision-detected" in first_line:                 # possible collision detected

        JSHOP_Plan.readline()                               # throw away second line  (read and do not use)

        new_direction_desired = (JSHOP_Plan.readline())[14] # get new desired aciton
        print "New desired action:", new_direction_desired

        if new_direction_desired == 'c':
            flag_direction = 1                              # go right flag
            print "Go right"
        elif new_direction_desired == 'a':
            flag_direction = 2                              # go left flag
            print "Go left"

if __name__ == '__main__':
    # JSHOP_to_ROS()
    try:
        airboat_guider()
    except rospy.ROSInterruptException:
        pass



