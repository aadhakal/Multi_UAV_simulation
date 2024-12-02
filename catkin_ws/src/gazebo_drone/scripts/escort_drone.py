#!/usr/bin/python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import cv2.aruco as aruco
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Connect to the escort drone
try:
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
    vehicle.parameters['PLND_ENABLED'] = 1
    vehicle.parameters['PLND_TYPE'] = 1
    vehicle.parameters['PLND_EST_TYPE'] = 0
    vehicle.parameters['LAND_SPEED'] = 30  # cm/s
except Exception as e:
    print("Error connecting to vehicle: " + str(e))
    exit(1)

# Camera intrinsics
np_camera_matrix = np.array([[530.8269276712998, 0.0, 320.5], [0.0, 530.8269276712998, 240.5], [0.0, 0.0, 1.0]])
np_distortion_coefficient = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# Waypoint details
takeoff_height = 5  # meters
wp_taco = LocationGlobalRelative(-35.36303741, 149.1652374, takeoff_height)

# ROS publishers
marker_pub = rospy.Publisher('/aruco_marker_location', String, queue_size=10)  # For target location
image_pub = rospy.Publisher('/drone1/camera/camera/color/image_new', Image, queue_size=10)  # For processed images

# ArUco setup
target_id = 72  # Target marker ID
marker_size = 20  # cm
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

# Function to arm and take off
def arm_and_takeoff(targetHeight):
    while not vehicle.is_armable:
        print("waiting for vehicle to be armable")
        time.sleep(1)
    print("vehicle is now armable")

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        print("waiting for drone to go to guided mode")
        time.sleep(1)

    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for the vehicle to be armed")
        time.sleep(1)
    print("vehicle is now armed")

    vehicle.simple_takeoff(targetHeight)

    while True:
        print("current Altitude: " + str(vehicle.location.global_relative_frame.alt))
        if vehicle.location.global_relative_frame.alt >= 0.95 * targetHeight:
            print("Target Altitude Reached")
            break
        time.sleep(1)

# Function to fly to a target location
def goto(targetLocation):
    vehicle.simple_goto(targetLocation)
    while vehicle.mode.name == 'GUIDED':
        distance_to_target = get_distance_meters(targetLocation, vehicle.location.global_relative_frame)
        if distance_to_target < 2:  # Stop when close enough
            print("Reached target waypoint")
            break
        time.sleep(1)

# Function to calculate distance between two GPS coordinates
def get_distance_meters(targetLocation, currentLocation):
    dLat = targetLocation.lat - currentLocation.lat
    dLon = targetLocation.lon - currentLocation.lon
    return math.sqrt((dLon * dLon) + (dLat * dLat)) * 1.113195e5

# Image processing callback
def msg_receiver(message):
    try:
        np_data = rnp.numpify(message)  # Convert ROS image to NumPy array
        gray_image = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
        (corners, ids, rejected) = aruco.detectMarkers(image=gray_image, dictionary=aruco_dict, parameters=parameters)

        if ids is not None:
            ids = ids.flatten()
            for i, marker_id in enumerate(ids):
                if marker_id == target_id:  # Check if the detected marker is the target
                    ret = aruco.estimatePoseSingleMarkers(corners[i], marker_size, np_camera_matrix, np_distortion_coefficient)
                    (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
                    x, y, z = tvec
                    print("Target Marker Found! ID=" + str(marker_id) + " Position: x=" + str(x) + " y=" + str(y) + " z=" + str(z))
                    
                    # Calculate GPS location of the marker
                    marker_lat = vehicle.location.global_relative_frame.lat + (x * 1e-6)
                    marker_lon = vehicle.location.global_relative_frame.lon + (y * 1e-6)
                    marker_location = str(marker_lat) + "," + str(marker_lon)
                    print("Publishing target marker location: " + marker_location)

                    # Publish target marker location
                    marker_pub.publish(marker_location)

                    # Annotate and publish processed image
                    aruco.drawDetectedMarkers(np_data, corners)
                    aruco.drawAxis(np_data, np_camera_matrix, np_distortion_coefficient, rvec, tvec, 10)
                    cv2.putText(np_data, "Marker ID: " + str(marker_id), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    new_msg = rnp.msgify(Image, np_data, encoding='rgb8')
                    image_pub.publish(new_msg)
                    return
        else:
            print("No markers detected in current frame")
    except Exception as e:
        print("Error processing image: " + str(e))

# Main function
if __name__ == "__main__":
    rospy.init_node("escort_drone_node", anonymous=False)
    sub = rospy.Subscriber("/drone1/camera/camera/color/image_raw", Image, msg_receiver)

    arm_and_takeoff(takeoff_height)
    goto(wp_taco)
    print("Reached waypoint, looking for target marker...")
    rospy.spin()
