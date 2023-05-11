'''
 * File: waypoints.py
 * State machine to go to several waypoints
'''

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool,CommandBoolRequest, SetMode,SetModeRequest, CommandTOL,CommandTOLRequest

# drone following vars
current_state = State()
current_pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_pose
    current_pose = msg

if __name__ == '__main__':
    rospy.init_node('offb_node_py')
    state_pub = rospy.Subscriber('/mavros/state',State,callback=state_cb)
    local_pose_pub = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=10)
    rospy.Subscriber('/mavros/local_position/pose',PoseStamped,callback=pose_cb)
    # arming service client
    rospy.wait_for_service('/mavros/cmd/arming')
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
    # set mode service client
    rospy.wait_for_service('/mavros/set_mode')
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode',SetMode)
    rospy.wait_for_service('/mavros/cmd/land')
    land_client = rospy.ServiceProxy('/mavros/cmd/land',CommandTOL)
    # data from initial takeoff
    initial_global_pos = rospy.wait_for_message('/mavros/global_position/global',NavSatFix)
    # land request var
    cmdTOL = CommandTOLRequest()
    cmdTOL.min_pitch = 0.0
    cmdTOL.yaw = 0.0
    cmdTOL.latitude = initial_global_pos.latitude
    cmdTOL.longitude = initial_global_pos.longitude
    cmdTOL.altitude = initial_global_pos.altitude
    print('init ready')
    
    # Setpoint publishing MUST be faster than 2Hz because PX4 timeout is 500ms
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while (not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
    print('Fight Controller connection ready')

    # waypoints
    waypoints = [(0.0,0.0,2.0),(5.0,5.0,2.0)]#,(2.0,2.0,2.0),(3.0,3.0,2.0),(0.0,0.0,2.0)]
    send_pose = PoseStamped()
    send_pose.pose.position.x = waypoints[0][0]
    send_pose.pose.position.y = waypoints[0][1]
    send_pose.pose.position.z = waypoints[0][2]
    current_wp = 1  # point to next waypoint
    print('waypoints ready')

    # Send a few setpoints before starting
    # You must have some streamed setpoints before if not the mode switch will reject
    for i in range(100):    # 100 is arbitrary
        if (rospy.is_shutdown()):
            break
        local_pose_pub.publish(send_pose)
        rate.sleep()
    print('setpoints sent')
    
    # service requests
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    # For not flooding the autopilot with requests
    last_req = rospy.Time.now()

    # state machine
    state = 0
    once = [True] * len(waypoints)
    dist_thresh = 0.05
    print('state machine configured')
    print('running main...')

    while(not rospy.is_shutdown()):
        # for checking if already in position
        x_err = current_pose.pose.position.x - send_pose.pose.position.x
        y_err = current_pose.pose.position.y - send_pose.pose.position.y
        z_err = current_pose.pose.position.z - send_pose.pose.position.z

        # state machine
        if state == 0:      # mode enable every 5 secs if not already in mode
            if(current_state.mode != 'OFFBOARD' and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo('OFFBOARD enable')
                last_req = rospy.Time.now()
            else:           # arming every 5 secs if not armed
                if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(arming_client.call(arm_cmd).success == True):
                        rospy.loginfo('Vehicle armed')
            # change state if armed and mode asked
            if current_state.mode == 'OFFBOARD' and current_state.armed:
                state = 2
            local_pose_pub.publish(send_pose)
            print('current_state:',current_state.mode,'armed:',current_state.armed)
        elif state == 2:        # go to waypoints through the list
            # check if already in position
            if abs(x_err) <= dist_thresh:   # check x
                print('x coords aligned',x_err)
                if abs(y_err) <= dist_thresh:   # check y
                    print('y coords aligned',y_err)
                    if abs(z_err) <= dist_thresh:   # check z
                        print('z aligned',z_err)
                        print('ALIGNED')
                        if current_wp < len(waypoints) and once[current_wp]:    # check if last waypoint and already checked
                            print('################ To Next Point ################')
                            # change to next waypoint
                            send_pose.pose.position.x = waypoints[current_wp][0]
                            send_pose.pose.position.y = waypoints[current_wp][1]
                            send_pose.pose.position.z = waypoints[current_wp][2]
                            once[current_wp] = False    # change only once
                            current_wp += 1
                        else:
                            state = 3   # go back to first point
            local_pose_pub.publish(send_pose)
        elif state == 3:    # go to initial point
            # check if already in point
            if abs(x_err) <= dist_thresh and abs(y_err) <= dist_thresh and abs(z_err) <= dist_thresh:
                # only once
                if once[0]:
                    state = 4
                    once[0] = False
            send_pose.pose.position.x = waypoints[0][0]
            send_pose.pose.position.y = waypoints[0][1]
            send_pose.pose.position.z = waypoints[0][2]
            local_pose_pub.publish(send_pose)
        elif state == 4:    # land command
            # try to land every 5 secs
            if(current_state.mode != 'LAND' and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(land_client.call(cmdTOL).success == True):
                    print('Landing...')
                    rospy.loginfo('LAND enable')
                last_req = rospy.Time.now()
        # local_pose_pub.publish(send_pose)
        print('current_pose: %.2f %.2f %.2f'%(current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z))
        print('send_pose:',send_pose.pose.position.x,send_pose.pose.position.y,send_pose.pose.position.z)
        print('state:%d'%state)
        rate.sleep()
