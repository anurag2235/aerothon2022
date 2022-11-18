#!/usr/bin/env python
# -*- coding: utf-8 -*-
# PRECISION NAVIGATION AND PAYLOAD DROP
# AEROKLE AEROTHON 2022
from __future__ import print_function
from email.mime import image
from geopy.distance import geodesic as GD
import time
from turtle import goto
from dronekit import connect, VehicleMode, LocationGlobalRelative,LocationGlobal,Command
from pymavlink import mavutil
import math
from functions import geta,getb,getc
import numpy as np     
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
connection_string = args.connect
sitl = None
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
# FOR PRECISION HOVERING
def send_body_ned_velocity2(velocity_x, velocity_y, velocity_z, duration=0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        0b0000111111000111, # type_mask
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # m/s
        0, 0, 0, # x, y, z acceleration
        0, 0)
    for x in range(0,duration):
         print(vehicle._location.global_relative_frame.alt)
         if vehicle.location.global_relative_frame.alt<1.5:
            break;
         else:     
          vehicle.send_mavlink(msg)
          time.sleep(1)                          
def drop_parcel():
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
        0,  # confirmation
        9,  # servo number
        1000,  # servo position between 1000 and 2000
        0, 0, 0, 0, 0)  # param 3 ~ 7 not used
    print("dropping parcel...")
    # send command to vehicle
    vehicle.send_mavlink(msg)
    print("parcel dropped...")     
#FOR PRECISION HOVERING AND PAYLOAD DROP
# FLIGHT COMMANDS
arm_and_takeoff(3)
point1 = LocationGlobalRelative(28.3677924,77.3163593,3)
vehicle.simple_goto(point1,groundspeed=5)
send_body_ned_velocity2(0,0,-1)
drop_parcel()
time.sleep(10)
vehicle.mode=VehicleMode("RTL")
exit()        