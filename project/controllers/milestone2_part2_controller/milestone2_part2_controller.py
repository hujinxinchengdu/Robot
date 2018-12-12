"""This is only a template controller which simply drives the car forward
"""

from vehicle import Driver

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

while driver.step() != -1:
    driver.setCruisingSpeed(40.0)
    driver.setBrakeIntensity(0)
    driver.setSteeringAngle(0.0)
