
import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import numpy as np

def navigate_to_waypoint(lat, lon, alt):
    
    print(f"Navigating to waypoint: Lat={lat}, Lon={lon}, Alt={alt}")
    waypoint = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(waypoint)

vehicle = connect('udp:10.87.52.89:14552', wait_ready=True)
print("Connected to MavProxy terminal")
target_altitude = 10

cam = cv2.VideoCapture(1)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
run = True
command = None
t = 0
while run:
    ret, frame = cam.read()
    if not ret or frame is None:
        print("Error: Could not read frame.")
        exit()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters) 
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1))
    
    rot, trans, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.04, camera_matrix, dist_coeffs)
    for i, corners in enumerate(corners):
        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rot[i], trans[i], 0.03)
        if (ids[i][0] == 0 and command == None):
            print(f"Waiting for mode change... (Current: {vehicle.mode.name})")
            ti = 0
            while vehicle.mode.name != "GUIDED" and ti<10:
                ti+=1
                time.sleep(1)
                if ti >= 10:
                    print("Timed out for mode change")
                    command = None         
                    break
            if ti < 10:
                command = 0
                print("Mode changed to GUIDED")
                
                while not vehicle.is_armable:
                    print("Waiting for vehicle to become armable...")
                    time.sleep(1)

                vehicle.armed = True
                while not vehicle.armed:
                    print("Waiting for arming...")
                    vehicle.armed = True
                    time.sleep(1)
                print("Taking off...")
                vehicle.simple_takeoff(target_altitude)
        
        if (ids[i][0] == 2 and command == None):
            command = 2
            navigate_to_waypoint(-35.3635, 149.1655, 10)
        if (ids[i][0] == 3 and command == None):
            command = 3
            navigate_to_waypoint(-35.3635, 149.1649, 10)
        if (ids[i][0] == 4 and command == None):
            command = 4
            navigate_to_waypoint(-35.3638, 149.1649, 10)
        if (ids[i][0] == 5 and command == None):
            command = 5
            navigate_to_waypoint(-35.3638, 149.1655, 10)
            
        if (ids[i][0] == 10 and command == None):
            print("Closing everything")
            vehicle.close()
            cv2.destroyAllWindows()
            quit()

    if command != None:
        t += 1
    if t > 100:
        t = 0
        command = None
        print("Command reset")
        
    cv2.imshow("ArUco Marker Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        run = False
        break
cv2.destroyAllWindows()