# drone take off, hover, and land

from dronekit import connect, VehicleMode
import time


def set_parameter(vehicle, param, value):
    try:
        vehicle.parameters[param] = value
        time.sleep(2)  # Allow time for the parameter to update
        print(f"Parameter {param} set to {value}.")
    except Exception as e:
        print(f"Error setting parameter {param}: {e}")



# Connection setup
print(f"Attempting to connect to vehicle on ..")
vehicle = connect("/dev/ttyAMA0",baud=115200,wait_ready=True)
print("Successfully connected to the vehicle.")



# Checking current mode
print("Checking current vehicle mode...")
print(f"Current Vehicle Mode: {vehicle.mode.name}")

"""home_location = vehicle.home_location
print(f"Home Location: Lat: {home_location.lat}, Lon: {home_location.lon}, Alt: {home_location.alt}")
"""

# Change mode to GUIDED
print("Attempting to change mode to GUIDED...")
vehicle.mode = VehicleMode("GUIDED")
while not vehicle.mode.name == 'GUIDED':
    print("Waiting for mode change to GUIDED...")
    time.sleep(1)
print("Vehicle mode successfully changed to GUIDED.")

# Arm the vehicle
print("Attempting to arm the vehicle...")
vehicle.armed = True
while not vehicle.armed:
    vehicle.armed = True
    print("Waiting for vehicle to arm...")
    time.sleep(1)
print("Vehicle successfully armed.")

# Takeoff
target_altitude = 10  # Target altitude in meters
print(f"Initiating takeoff to {target_altitude} meters...")
vehicle.simple_takeoff(target_altitude)

# Monitor altitude during takeoff
print("Monitoring altitude during takeoff...")
while True:
    current_altitude = vehicle.location.global_relative_frame.alt
    print(f"Current Altitude: {current_altitude} meters")
    if current_altitude >= target_altitude * 0.95:  # Allowing a small margin
        print(f"Reached target altitude of {target_altitude} meters.")
        break
    time.sleep(1)

# Hover for 15 seconds
hover_time = 15  # Hover duration in seconds
print(f"Hovering at {target_altitude} meters for {hover_time} seconds...")
hover_start = time.time()
while time.time() - hover_start < hover_time:
    print(f"Hovering... Time elapsed: {int(time.time() - hover_start)} seconds")
    time.sleep(1)
print(f"Completed hover duration of {hover_time} seconds.")

# Land the vehicle
print("Initiating landing sequence...")
vehicle.mode = VehicleMode("LAND")
while vehicle.location.global_relative_frame.alt > 0.5:
    print(f"Descending... Current Altitude: {vehicle.location.global_relative_frame.alt} meters")
    time.sleep(1)
print("Landing complete. Vehicle is on the ground.")

# Close the connection
print("Closing vehicle connection...")
vehicle.close()
print("Mission complete.")
