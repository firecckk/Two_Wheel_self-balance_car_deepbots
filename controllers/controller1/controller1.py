
import sys
import matplotlib.pyplot as plt
import numpy as np
from controller import Robot, AnsiCodes
from simple_pid import PID

INERTIAL_UNIT = 'inertial unit'
GYRO_NAME = 'gyro'
ACC_NAME = 'accelerometer'
TIME_STEP = 4
SPEED_LIMIT = 600

class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = TIME_STEP
        self.inertia = self.getDevice(INERTIAL_UNIT)
        self.gyro = self.getDevice(GYRO_NAME)
        self.acc = self.getDevice(ACC_NAME)
        self.motorL = self.getDevice('motor_right')
        self.motorR = self.getDevice('motor_left')

        self.inertia.enable(self.timeStep)
        #self.gyro.enable(self.timeStep)
        #self.acc.enable(self.timeStep)
        # allow motor speed control
        self.motorL.setPosition(float('inf'))
        self.motorR.setPosition(float('inf'))
        self.motorL.setVelocity(0)
        self.motorR.setVelocity(0)

        self.pid_upRight = PID(60, 0, 2.5, setpoint=0) # PD controller
        self.pid_speed = PID(0.68, 1.3, 0, setpoint=0) # PI controller

        self.output_figure = Figure("plot.png")
        self.err_figure = Figure("err.png")

    def steps(self):
        while self.step(self.timeStep) != -1:
            #gx, gy, gz = self.gyro.getValues()
            print(AnsiCodes.CLEAR_SCREEN)
            #print(f'rotation axes: [ x y z ] = [ {vel[0]:+.2f} {vel[1]:+.2f} {vel[2]:+.2f} ]')
            #ax, ay, az = self.acc.getValues()
            #print(f'acceleratoin: [ x y z ] = [ {vel[0]:+.2f} {vel[1]:+.2f} {vel[2]:+.2f} ]')
            roll, pitch, yaw = self.inertia.getRollPitchYaw()
            print(f'roll:{roll:+.2f} pitch:{pitch:+.2f} yaw:{yaw:+.2f}')

            target_angle = 0
            total_speed = self.motorL.getVelocity() + self.motorR.getVelocity()
            err = target_angle - pitch
            result_upright = self.pid_upRight(err)
            result_speed = self.pid_speed(total_speed)

            output_speed = result_speed + result_upright

            if(abs(output_speed)>SPEED_LIMIT):
                output_speed = SPEED_LIMIT
            self.motorL.setVelocity(-output_speed)
            self.motorR.setVelocity(-output_speed)
            print(f'error: {err:+.2f} speed:{total_speed:+.2f} pid: {result_speed:+.3f} out: {-output_speed}')
            self.output_figure.update(self.getTime(), -output_speed)
            self.err_figure.update(self.getTime(), err)
        #self.output_figure.save()
        #self.err_figure.save()
        sys.exit()

    def run(self):
        self.steps()

class Figure():
    def __init__(self, name):
        self.fig, self.ax = plt.subplots()
        self.x = []
        self.y = []
        self.name = name

    def update(self, new_x, new_y):
        self.x.append(new_x)
        self.y.append(new_y)
    
    def save(self):
        self.ax.plot(self.x, self.y)
        self.fig.savefig(self.name)


controller = Controller()
controller.run()