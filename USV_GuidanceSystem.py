#!/usr/bin/env python
# license removed for brevity

import  rospy
import  os
import  time
from    std_msgs.msg import String
from    sensor_msgs.msg import JointState
from    gazebo_msgs.msg import ModelStates


####################
# Global Variables #
####################
flag_direction  = 0                                                             # flag for USV direction decision
string          = '        (at         intruder    GC_B3       )        \n'     # information about collision
evaded          = 0
counter         = 0
counter2        = 0
counter3        = 0
counter4        = 0

def ros_feedback_callback(data):
    
    global evaded
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose[3].position.x)
    # rospy.loginfo("Airboat  x: %d y: %d", data.pose[1].position.x, data.pose[1].position.y)
    # rospy.loginfo("Buyo     x: %d y: %d", data.pose[3].position.x, data.pose[3].position.y)

    models_position_airboat     = 1
    models_position_intruder    = 2
    
    if ((abs(data.pose[models_position_intruder].position.x - data.pose[models_position_airboat].position.x) < 8) and 
        (abs(data.pose[models_position_intruder].position.y - data.pose[models_position_airboat].position.y) < 8)):

        if evaded == 0:

            # print "Near"
        
            # Copy file content as a string vector
            f           = open("/home/darlan/Darlan/Projects/Artificial-Inteligence/Planning/Hierarchical-Task-Network/jshop2/examples/colregs/problem.pddl", "r")
            content     = f.readlines()
            f.close()

            # print content

            # # Prepare for overwriting last 6 lines of the file with new information about possible collision
            # print content[-6]
            content[-6] = string
            # print content[-6]

            # Write changes on file (overwriting)
            f           = open("/home/darlan/Darlan/Projects/Artificial-Inteligence/Planning/Hierarchical-Task-Network/jshop2/examples/colregs/problem.pddl", "w")
            for line in content:
                f.write(line)
            f.close()

            os.chdir("/home/darlan/Darlan/Projects/Artificial-Inteligence/Planning/Hierarchical-Task-Network/jshop2")
            os.system("make colregs")

            evaded = 1

def airboat_guider():

    global flag_direction
    global string
    global evaded

    # os.chdir("/home/darlan/Darlan/Projects/Artificial-Inteligence/Planning/Hierarchical-Task-Network/jshop2")
    # os.system("make colregs")

    # ROS node initialization
    rospy.init_node('airboat_guider', anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, ros_feedback_callback)

    # Publishers for joint turning and thruster control
    pub_joint                   = rospy.Publisher('/airboat/joint_states',      JointState, queue_size=10)
    pub_thruster                = rospy.Publisher('/airboat/thruster_command',  JointState, queue_size=10)

    # Publish freqeuncy for both msgs (joint and thruster)
    rate                        = rospy.Rate(1000) # 10hz

    # Handle for publishing
    joint_state_msg             = JointState()
    thruster_command_msg        = JointState()

    # Values to be published
    joint_position_vector       = [0]
    thruster_position_vector    = [0]

    # counter for JSHOP parser
    counter = 0

    while not rospy.is_shutdown():

        # read JSHOP plan each 5 seconds
        counter = counter +1
        if counter == 1000*5:
            counter = 0
            JSHOP_to_ROS()

        # boat direction control
        if flag_direction == 0:
            thruster_position_vector    = [3]   # go ahead
            joint_position_vector       = [0]   # align to center
            # print "go ahead"

        elif flag_direction == 1:
            thruster_position_vector    = [2]       # go ahead
            joint_position_vector       = [-0.5]      # turn right
            # print "go right"

        elif flag_direction == 2:
            thruster_position_vector    = [1]       # go ahead
            joint_position_vector       = [1]       # turn left
            # print "go left"

        elif flag_direction == 3:
            thruster_position_vector    = [4]       # go ahead
            joint_position_vector       = [-0.3]       # turn right

        elif flag_direction == 4:
            thruster_position_vector    = [2]       # go ahead
            joint_position_vector       = [0.2]    # turn right

        joint_state_msg.name            = ['fwd_joint']
        joint_state_msg.position        = joint_position_vector

        thruster_command_msg.name       = ['fwd']
        thruster_command_msg.position   = thruster_position_vector

        pub_joint.publish(joint_state_msg)
        pub_thruster.publish(thruster_command_msg)

        rate.sleep()

def JSHOP_to_ROS():

    global flag_direction
    global counter
    global counter2
    global counter3
    global counter4
    
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
            counter = counter +1
            if counter < 4:
                flag_direction = 1                          # go right flag
            else:
                print "Must be going straight"
                counter2 = counter2 +1
                if counter2 < 3:
                    flag_direction = 2                      # go left flag
                else:
                    counter3 = counter3 +1
                    if counter3 < 3:
                        flag_direction = 3                  # go left flag
                    else:
                        counter4 = counter4 +1
                        if counter4 < 3:
                            flag_direction = 4
                        else:
                            flag_direction = 0
                    

            # print "Go right"
        elif new_direction_desired == 'a':
            flag_direction = 2                              # go left flag
            # print "Go left"
        else: 
            flag_direction = 0
            # print "Go ahead"

    JSHOP_Plan.close()

if __name__ == '__main__':
    try:
        airboat_guider()
    except rospy.ROSInterruptException:
        pass
