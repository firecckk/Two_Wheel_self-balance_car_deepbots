from controller import Robot, DistanceSensor

TIME_STEP = 32

robot = Robot()

sensor = robot.getDevice("MPU9250")
sensor.enable(TIME_STEP)

while robot.step(TIME_STEP) != -1:
    value = sensor.getValue()
    print("Sensor value is: ", value)