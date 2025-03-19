from dronekit import connect
import time

def armandtakeoff(altitude):
    timeelapsed=0
    print("ARMING")
    print("Changing Mode to Guided")
    vehicle.mode="GUIDED"
    vehicle.armed=True
    while not vehicle.armed:
        print("Waiting for ARM")
        time.sleep(1)
    print("ARMED")
    time.sleep(2)
    print("TAKING OFF")
    time.sleep(1)
    vehicle.simple_takeoff(10)

    while True:
        print("Altitude:"+str(vehicle.location.global_relative_frame.alt))
        time.sleep(1)
        if (vehicle.location.global_relative_frame.alt>=altitude*0.98):
            break

    hover_time=10
    t=time.time()
    print(f"Hovering at {altitude}m for {hover_time} seconds")
    while (time.time()-t)< hover_time:
        print("Time Elapsed:"+str(time.time()-t))
        time.sleep(0.5)
    print("Landing")
    vehicle.mode="LAND"

print("Connecting To Vehicle........")
vehicle=connect("udp:0.0.0.0:14550",wait_ready=True)
if vehicle:
    print("Connection Succesfull")
    armandtakeoff(10)
else:
    print("Connection Cannot be Established")

