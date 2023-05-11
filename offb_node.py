'''
 * File: offb_node.py
 * Stack and tested in Gazebo Classic  SITL
'''

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

if __name__ == '__main__':
    rospy.init_node('offb_node_py')
    state_pub = rospy.Subscriber('/mavros/state',State,callback=state_cb)
    local_pose_pub = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=10)
    rospy.wait_for_service('/mavros/cmd/arming')
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
    rospy.wait_for_service('/mavros/set_mode')
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode',SetMode)
    
    # Setpoint publishing MUST be faster than 2Hz because PX4 timeout is 500ms
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while (not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
    
    pose = PoseStamped()
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = 2.0

    # Send a few setpoints before starting
    # You must have some streamed setpoints before if not the mode switch will reject
    for i in range(100):    # 100 is arbitrary
        if (rospy.is_shutdown()):
            break
        local_pose_pub.publish(pose)
        rate.sleep()
    
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    # For not flooding the autopilot with requests
    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != 'OFFBOARD' and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo('OFFBOARD enable')
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo('Vehicle armed')
        
        local_pose_pub.publish(pose)
        rate.sleep()
