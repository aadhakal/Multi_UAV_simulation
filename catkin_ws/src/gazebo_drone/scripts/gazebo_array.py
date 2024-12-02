#!/usr/bin/python

import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from array import array

vehicle = connect('udp:127.0.0.1:14550',wait_ready=True)
vehicle.parameters['PLND_ENABLED']=1
vehicle.parameters['PLND_TYPE']=1
vehicle.parameters['PLND_EST_TYPE']=0
vehicle.parameters['LAND_SPEED']=30 #cm/s

velocity = .5 #m/s
takeoff_height = 9 #m 

newimage_pub = rospy.Publisher('camera/color/image_new', Image, queue_size=10)

ids_to_find = [129,72]
marker_sizes = [40,20]
marker_heights = [10,4]

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 640 # in pixels
vertical_resolution = 480  #in pixels

horizontal_fov = 62.2 * (math.pi / 180) #depends on a pi cam version
vertical_fov = 48.8 * 2 * (math.pi / 180)

found_count = 0
not_found_count = 0
time_last = 0
time_to_wait = .1 #100ms

# Camera Intrinics
distortion_coefficient = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix  = [[530.8269276712998, 0.0, 320.5], [0.0, 530.8269276712998, 240.5], [0.0, 0.0, 1.0]] #in real world we need to find the actual value fo our camera
np_camera_matrix = np.array(camera_matrix)
np_distortion_coefficient = np.array(distortion_coefficient)

def arm_and_takeoff(targetHeight):
    while vehicle.is_armable != True:
        print('waiting for vehicle to be armable')
        time.sleep(1)
    print ('vehicle is now available')

    vehicle.mode = VehicleMode('GUIDED')
    while vehicle.mode != 'GUIDED':
        print('waiting for drone to go to guided flight mode')
        time.sleep(1)
    print('vehicle is now in guided mode')

    vehicle.armed = True
    while vehicle.armed == False:
        print('Waiting for the vehicle to be armed')
        time.sleep(1)
    print('vehicle is now armed')

    vehicle.simple_takeoff(targetHeight)

    while True:
        print('current Altitude:%d'%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= .95 * targetHeight:
            break
        time.sleep(1)
    print('Target Altitude Reached')

    return None

def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0,
        0,
        0,
        vx,
        vy,
        vz,
        0,0,0,0,0
        )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def send_landing_message(x, y):  # distance between the drone and the target
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,0,0
        )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def msg_receiver(message):
    global not_found_count, found_count, time_last, time_to_wait, id_to_find

    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(message) # deserialize image Data into array
        gray_image = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(image=gray_image, dictionary=aruco_dict, parameters=parameters)

        altitude = vehicle.location.global_relative_frame.alt # in meters

        id_to_find = 0
        marker_height = 0
        marker_size = 0

        if altitude > marker_heights[1]:
            id_to_find = ids_to_find[0]
            marker_height = marker_heights[0]
            marker_size = marker_sizes[0]
        elif altitude < marker_heights[1]:
            id_to_find = ids_to_find[1]
            marker_height = marker_heights[1]
            marker_size = marker_sizes[1]
        
        ids_array_index = 0
        found_id = 0
        print("Looking for marker: " + str(id_to_find))

        try:
            if ids is not None:
                for id in ids:
                    if id == id_to_find:
                        corners_single = [corners[ids_array_index]]
                        corners_single_np = np.asarray(corners_single)

                        ret= aruco.estimatePoseSingleMarkers(corners_single, marker_size, cameraMatrix = np_camera_matrix, distCoeffs = np_distortion_coefficient)
                        (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                        x = '{:.2f}'.format(tvec[0])  #distance etween camera in aruco in CM
                        y = '{:.2f}'.format(tvec[1])
                        z = '{:.2f}'.format(tvec[2])

                        x_sum = 0
                        y_sum = 0
                        x_avg = 0
                        y_avg = 0

                        x_sum = corners_single_np[0][0][0][0] + corners_single_np[0][0][1][0] + corners_single_np[0][0][2][0] + corners_single_np[0][0][3][0] # pixel value avg from corners array
                        y_sum = corners_single_np[0][0][0][1] + corners_single_np[0][0][1][1] + corners_single_np[0][0][2][1] + corners_single_np[0][0][3][1]

                        x_avg = x_sum / 4
                        y_avg = y_sum / 4

                        x_ang = (x_avg - horizontal_res * .5) * (horizontal_fov / horizontal_res) # gives the center position of the Aruco
                        y_ang = (y_avg - vertical_resolution * .5) * (vertical_fov / vertical_resolution)
                    
                        if vehicle.mode != 'LAND':
                            vehicle.mode = VehicleMode('LAND')
                            while vehicle.mode != 'LAND':
                                time.sleep(1)
                            print('Vehicle is in LAND mode')
                            send_landing_message(x_ang, y_ang)
                        else:
                            send_landing_message(x_ang, y_ang)

                        marker_position = 'MARKER POSITION: x = ' +x+' y = '+y+' z = '+z

                        aruco.drawDetectedMarkers(np_data, corners)
                        aruco.drawAxis(np_data, np_camera_matrix, np_distortion_coefficient, rvec, tvec, 10)
                        cv2.putText(np_data, marker_position, (10,50),0,.7, (255,0,0), thickness = 2)
                        print(marker_position)
                        print('FOUND COUNT: ' + str(found_count) + ' NOT FOUND COUNT: ' + str(not_found_count))
                        found_count = found_count + 1
                        found_id = 1
                        break
                    ids_array_index = ids_array_index + 1
                if found_id == 0:
                    not_found_count = not_found_count + 1
            else:
                not_found_count = not_found_count + 1    
        except Exception as e:
            print('Target Likely Not Found')
            print(e)
            not_found_count = not_found_count + 1
        new_msg = rnp.msgify(Image, np_data, encoding='rgb8') # converts np message to ROS message Format
        newimage_pub.publish(new_msg)
        time_last = time.time()
    else:
        return None


def subscriber():
    rospy.init_node('drone_node', anonymous=False)
    sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)
    rospy.spin()


if __name__ == '__main__':
    try:
        arm_and_takeoff(takeoff_height)
        time.sleep(1)
        send_local_ned_velocity(velocity, 0 , 0)
        time.sleep(10)
        subscriber()
    except rospy.ROSInterruptException:
        pass





