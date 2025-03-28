import time
from dronekit import connect, VehicleMode, Command,LocationGlobalRelative,APIException
from pymavlink import mavutil  # For sending velocity commands
import socket
import exceptions
import math
import argparse
# Connect to the vehicle (replace connection_string with your SITL connection)
# connection_string = "tcp:127.0.0.1:5762"  # Default SITL connection
# print(f"Connecting to vehicle on: {connection_string}")
# vehicle = connect(connection_string, wait_ready=True)


def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()


    connection_string = args.connect
    baud_rate = 57600

    vehicle = connect(connection_string,baud = baud_rate,wait_ready= True)
    return vehicle
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Send velocity commands in the NED frame (forward, right, down).
    velocity_x: Forward speed (m/s, positive = forward)
    velocity_y: Right speed (m/s, positive = right)
    velocity_z: Down speed (m/s, positive = down)
    duration: Time to apply the velocity (seconds)
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)     # yaw, yaw_rate (not used)
    
    # Send command for the specified duration
    for _ in range(0, int(duration)):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def arm_and_takeoff(target_altitude):
    vehicle = connectMyCopter()
    print("Basic pre-arm checks...")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    print("ARMED")
    time.sleep(3)
    print(f"Taking off to {target_altitude} meters!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_altitude:.2f} m")
        if current_altitude >= target_altitude * 0.95:
            print("Reached target altitude!")
            break
        time.sleep(1)

    # Hover for 5 seconds
    print("Hovering for 5 seconds...")
    time.sleep(5)

   
    print("Pitching forward (moving forward at 5 m/s)...")
    send_ned_velocity(1, 0, 0, 2)  

    
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.location.global_relative_frame.alt > 0.1:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f} m")
        time.sleep(1)
    print("Landed!")


try:
    arm_and_takeoff(1.5)  
finally:
    print("Closing vehicle connection...")
    vehicle.close()
