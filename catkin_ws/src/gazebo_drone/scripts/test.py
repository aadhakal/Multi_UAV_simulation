from dronekit import connect, VehicleMode
import time

def connect_vehicle(connection_string):
    """
    Connect to a vehicle using the specified connection string.
    """
    print("Connecting to vehicle on: " + connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    return vehicle

def program_drone(vehicle, target_altitude):
    """
    Program a single drone to take off and reach a target altitude.
    """
    print("Arming the drone...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print("Altitude: " + str(vehicle.location.global_relative_frame.alt))
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

    # Example task: Hover for 10 seconds
    print("Hovering...")
    time.sleep(10)

    print("Landing...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.location.global_relative_frame.alt > 0.1:
        print("Landing... Altitude: " + str(vehicle.location.global_relative_frame.alt))
        time.sleep(1)

    print("Landed. Disarming.")
    vehicle.armed = False
    vehicle.close()

def main():
    # Connection strings for each drone
    drone1_connection = "127.0.0.1:14554"
    drone2_connection = "127.0.0.1:14555"

    # Connect to both drones
    drone1 = connect_vehicle(drone1_connection)
    drone2 = connect_vehicle(drone2_connection)

    try:
        # Program Drone 1 and Drone 2 in parallel
        print("Programming Drone 1...")
        program_drone(drone1, target_altitude=10)

        print("Programming Drone 2...")
        program_drone(drone2, target_altitude=15)
    except KeyboardInterrupt:
        print("Mission interrupted. Returning drones to LAND mode.")
        drone1.mode = VehicleMode("LAND")
        drone2.mode = VehicleMode("LAND")
    finally:
        print("Closing vehicle connections.")
        drone1.close()
        drone2.close()

if __name__ == "__main__":
    main()
