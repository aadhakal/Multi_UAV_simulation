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

newimage_pub = rospy.Publisher('drone1/camera/camera/color/image_new', Image, queue_size=10)

id_to_find = 129 

marker_size  = 20 #in CM

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_resolution = 640 # in pixels
vertical_resolution = 480  #in pixels

horizonatl_fov = 62.2 * (math.pi / 180) #depends on a pi cam version
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


def msg_receiver(message):
    global not_found_count, found_count, time_last, time_to_wait, id_to_find

    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(message) # deserialize image Data into array
        gray_image = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(image=gray_image, dictionary=aruco_dict, parameters=parameters)
        try:
            if ids is not None:
                if ids[0] == id_to_find:
                    ret= aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix = np_camera_matrix, distCoeffs = np_distortion_coefficient)
                    (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                    x = '{:.2f}'.format(tvec[0])  #distance etween camera in aruco in CM
                    y = '{:.2f}'.format(tvec[1])
                    z = '{:.2f}'.format(tvec[2])

                    marker_position = 'MARKER POSITION: x = ' +x+' y = '+y+' z = '+z

                    aruco.drawDetectedMarkers(np_data, corners)
                    aruco.drawAxis(np_data, np_camera_matrix, np_distortion_coefficient, rvec, tvec, 10)
                    cv2.putText(np_data, marker_position, (10,50),0,.7, (255,0,0), thickness = 2)
                    found_count = found_count + 1
                    print(marker_position)
                else:
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
    sub = rospy.Subscriber('drone1/camera/camera/color/image_raw', Image, msg_receiver)
    rospy.spin()


if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass




