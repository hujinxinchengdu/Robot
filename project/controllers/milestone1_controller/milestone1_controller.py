"""This is only a template controller which simply drives the car forward
"""

from vehicle import Driver
import math

driver = Driver()

# An example of how you get a sensor and initialize it.
# You are free to use all the sensors that comes with your car.
# Check Sensor Slot fields on the vehicle in the left pane of Webots for available sensors.
#front_sensor = driver.getDistanceSensor("distance sensor front")
#front_sensor.enable(16)

lidar = driver.getLidar("Velodyne HDL-64E")
lidar.enable(160)
lidar.enablePointCloud()

camera = driver.getCamera("camera")
camera.enable(32)

count = 0
state = 0
threshold = 0.007

lidar_point_cloud_14 = []
lidar_point_cloud_15 = []

while driver.step() != -1:
    count = count + 1
    if count % 10 == 0:
        d1 = 0
        d2 = 0
        lidar_point_cloud_14 = lidar.getLayerPointCloud(32)
        lidar_point_cloud_15 = lidar.getLayerPointCloud(33)
       
        for i in range(0,140):
            #d1 = d1 + math.sqrt((lidar_point_cloud_14[i].x-lidar_point_cloud_15[i].x) ** 2 + (lidar_point_cloud_14[i].z-lidar_point_cloud_15[i].z) ** 2) 
            #d2 = d2 + math.sqrt((lidar_point_cloud_14[i + 1025].x-lidar_point_cloud_15[i + 1025].x) ** 2 + (lidar_point_cloud_14[i + 1025].z-lidar_point_cloud_15[i + 1025].z) ** 2) 
            d1 = d1 + abs(lidar_point_cloud_14[i].y-lidar_point_cloud_15[i].y)
            d2 = d2 + abs(lidar_point_cloud_14[i + 984].y-lidar_point_cloud_15[i + 984].y)
        d1_mean = d1 / 141
        d2_mean = d2 / 141
        if (d2_mean - d1_mean) > 0.00585:
        #0.0058
            print("1")
            state = 1
        elif (d1_mean - d2_mean) > 0.00585:
            print("2")
            state = 2
        else:
            print("0")
            state = 0
        print(d1_mean - d2_mean)
        # 33 34 threshold = 0.007
        
        # 14，15
        #if abs(lidar_point_cloud_14[0].z - lidar_point_cloud_15[0].z) < 0.1 and abs(lidar_point_cloud_14[0].z - lidar_point_cloud_15[0].z)!=0:
            #state = 1   
        #else:
            #state = 0


    if state == 0:        
        driver.setCruisingSpeed(15.0) 
        driver.setBrakeIntensity(0)
        driver.setSteeringAngle(0.0)
    elif state == 1:
        driver.setCruisingSpeed(8.0)
        driver.setBrakeIntensity(1.0)
        driver.setSteeringAngle(35)
    else:
        driver.setCruisingSpeed(8.0)
        driver.setBrakeIntensity(1.0)
        driver.setSteeringAngle(-35)
        