#!/usr/bin/python

import rospy
from std_msgs.msg import String
from dronekit import connect, VehicleMode, LocationGlobalRelative
import math
import time

# Connect to the delivery drone
try:
    print("Connecting to delivery drone...")
    vehicle = connect('udp:127.0.0.1:14551', wait_ready=True, timeout=60)
    print("Connected to delivery drone")
except Exception as e:
    print("Error connecting to vehicle: " + str(e))
    exit(1)

# Waypoint details
takeoff_height = 5  # meters
marker_lat = None
marker_lon = None

# Function to arm and take off
def arm_and_takeoff(targetHeight):
    while not vehicle.is_armable:
        print("Waiting for vehicle to be armable")
        time.sleep(1)
    print("Vehicle is now armable")

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        print("Waiting for drone to go to guided mode")
        time.sleep(1)

    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for the vehicle to be armed")
        time.sleep(1)
    print("Vehicle is now armed")

    vehicle.simple_takeoff(targetHeight)

    while True:
        print("Current Altitude: " + str(vehicle.location.global_relative_frame.alt))
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

# Callback for receiving marker location
def marker_location_callback(data):
    global marker_lat, marker_lon
    try:
        # Parse received marker location
        marker_lat, marker_lon = map(float, data.data.split(","))
        print("Received marker location: lat=" + str(marker_lat) + ", lon=" + str(marker_lon))
        
        # Navigate to the marker location
        wp_marker = LocationGlobalRelative(marker_lat, marker_lon, takeoff_height)
        goto(wp_marker)

        # Simulate payload delivery
        print("Payload delivered!")
        time.sleep(2)

        # Land the drone
        vehicle.mode = VehicleMode("LAND")
        while vehicle.mode != "LAND":
            print("Waiting for drone to switch to LAND mode")
            time.sleep(1)
        print("Drone is landing...")
    except Exception as e:
        print("Error in marker location callback: " + str(e))

# Main function
if __name__ == "__main__":
    try:
        # Initialize ROS node
        rospy.init_node("delivery_drone_node", anonymous=False)

        # Subscribe to the target marker location topic
        rospy.Subscriber("/aruco_marker_location", String, marker_location_callback)

        # Arm and take off to a safe altitude
        arm_and_takeoff(takeoff_height)
        print("Waiting for target marker location...")

        # Keep the node alive
        rospy.spin()
    except rospy.ROSInterruptException:
        print("ROS Interrupt Exception")
    except Exception as e:
        print("An error occurred: " + str(e))
    finally:
        # Safely close the vehicle connection
        if vehicle:
            vehicle.close()
            print("Vehicle connection closed")
