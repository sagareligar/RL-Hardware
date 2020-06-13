one_degree = 0.0174532    # 2pi/360
six_degrees = 0.1047192
twelve_degrees = 0.2094384
fifty_degrees = 0.87266

import time
from Kalman import KalmanAngle
import smbus      #import SMBus module of I2C
# import time
import math
import RPi.GPIO as GPIO

kalmanX = KalmanAngle()
kalmanY = KalmanAngle()

RestrictPitch = False 
radToDeg = 57.2957786
kalAngleX = 0
kalAngleY = 0
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47



in1 = 24
in2 = 23
en = 25


in3 = 15
in4 = 14
en1 = 18

temp1=1

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en,GPIO.OUT)


GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(en1,GPIO.OUT)



GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)

GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)


p=GPIO.PWM(en,1000)
q=GPIO.PWM(en1,1000)

p.start(25)
q.start(25)

#Read the gyro and acceleromater values from MPU6050
def MPU_Init():
  #write to sample rate register
  bus.write_byte_data(DeviceAddress, SMPLRT_DIV, 7)

  #Write to power management register
  bus.write_byte_data(DeviceAddress, PWR_MGMT_1, 1)

  #Write to Configuration register
  #Setting DLPF (last three bit of 0X1A to 6 i.e '110' It removes the noise due to vibration.) https://ulrichbuschbaum.wordpress.com/2015/01/18/using-the-mpu6050s-dlpf/
  bus.write_byte_data(DeviceAddress, CONFIG, int('0000110',2))

  #Write to Gyro configuration register
  bus.write_byte_data(DeviceAddress, GYRO_CONFIG, 24)

  #Write to interrupt enable register
  bus.write_byte_data(DeviceAddress, INT_ENABLE, 1)


def read_raw_data(addr):
  #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(DeviceAddress, addr)
        low = bus.read_byte_data(DeviceAddress, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
DeviceAddress = 0x68   # MPU6050 device address

MPU_Init()

time.sleep(1)
#Read Accelerometer raw value
accX = read_raw_data(ACCEL_XOUT_H)
accY = read_raw_data(ACCEL_YOUT_H)
accZ = read_raw_data(ACCEL_ZOUT_H)

#print(accX,accY,accZ)
#print(math.sqrt((accY**2)+(accZ**2)))
if (RestrictPitch):
    roll = math.atan2(accY,accZ) * radToDeg
    pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
else:
    roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
    pitch = math.atan2(-accX,accZ) * radToDeg
print(roll)
kalmanX.setAngle(roll)
kalmanY.setAngle(pitch)
gyroXAngle = roll;
gyroYAngle = pitch;
compAngleX = roll;
compAngleY = pitch;

timer = time.time()
flag = 0


class ReinforcementLearner():

    # initialize a new ReinforcementLearner with some default parameters
  def __init__(self,alpha=1000, beta=0.5, gamma=0.95, lambda_w=0.9, lambda_v=0.8, max_failures=50, max_steps=1000000, max_distance=2.4, max_speed=1, max_angle_factor=12):
    self.n_states = 162         # 3x3x6x3 = 162 states
    self.alpha = alpha          # learning rate for action weights
    self.beta = beta            # learning rate for critic weights
    self.gamma = gamma          # discount factor for critic
    self.lambda_w = lambda_w    # decay rate for action weights
    self.lambda_v = lambda_v    # decay rate for critic weights
    self.max_failures = max_failures
    self.max_steps = max_steps

    self.max_distance = max_distance
    self.max_speed = max_speed
    self.max_angle = max_angle_factor * one_degree

    self.action_weights = [0] * self.n_states    # action weights
    self.critic_weights = [0] * self.n_states    # critic weights
    self.action_weights_elig = [0] * self.n_states    # action weight eligibilities
    self.critic_weights_elig = [0] * self.n_states # critic weight eligibilities

    #position, velocity, angle, angle velocity
    self.x, self.dx, self.t, self.dt = 0, 0, 0, 0
    
  def get_state(self):
    state = 0

    # failed
    if self.x < -self.max_distance or self.x > self.max_distance or self.t < -self.max_angle or self.t > self.max_angle:
      return -1

    #position
    if self.x < -0.8:
      state = 0
    elif self.x < 0.8:
      state = 1
    else:
      state = 2

    #velocity
    if self.dx < -self.max_speed:
      state += 0
    elif self.dx < 0.5:
      state += 3
    else:
      state += 6

    #angle
    if self.t < -six_degrees:
      state += 0
    elif self.t < -one_degree:
      state += 9
    elif self.t < 0:
      state += 18
    elif self.t < one_degree:
      state += 27
    elif self.t < six_degrees:
      state += 36
    else:
      state += 45

    #angle velocity
    if self.dt < -fifty_degrees:
      state += 0
    elif self.dt < fifty_degrees:
      state += 54
    else:
      state += 108

    return state


  def read_variables(self):
    #self.x = self.controller.get_current_position()
    #self.dx = self.controller.get_current_ground_speed()
 



      #Read Accelerometer raw value
      accX = read_raw_data(ACCEL_XOUT_H)
      accY = read_raw_data(ACCEL_YOUT_H)
      accZ = read_raw_data(ACCEL_ZOUT_H)

      #Read Gyroscope raw value
      gyroX = read_raw_data(GYRO_XOUT_H)
      gyroY = read_raw_data(GYRO_YOUT_H)
      gyroZ = read_raw_data(GYRO_ZOUT_H)

      dt = time.time() - timer
      timer = time.time()

      if (RestrictPitch):
          roll = math.atan2(accY,accZ) * radToDeg
          pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
      else:
          roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
          pitch = math.atan2(-accX,accZ) * radToDeg

      gyroXRate = gyroX/131
      gyroYRate = gyroY/131

      if (RestrictPitch):

          if((roll < -90 and kalAngleX >90) or (roll > 90 and kalAngleX < -90)):
              kalmanX.setAngle(roll)
              complAngleX = roll
              kalAngleX   = roll
              gyroXAngle  = roll
          else:
              kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

          if(abs(kalAngleX)>90):
              gyroYRate  = -gyroYRate
              kalAngleY  = kalmanY.getAngle(pitch,gyroYRate,dt)
      else:

          if((pitch < -90 and kalAngleY >90) or (pitch > 90 and kalAngleY < -90)):
              kalmanY.setAngle(pitch)
              complAngleY = pitch
              kalAngleY   = pitch
              gyroYAngle  = pitch
          else:
              kalAngleY = kalmanY.getAngle(pitch,gyroYRate,dt)

          if(abs(kalAngleY)>90):
              gyroXRate  = -gyroXRate
              kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

    #angle = (rate of change of angle) * change in time
      gyroXAngle = gyroXRate * dt
      gyroYAngle = gyroYAngle * dt

    #compAngle = constant * (old_compAngle + angle_obtained_from_gyro) + constant * angle_obtained from accelerometer
      compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll
      compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch

      if ((gyroXAngle < -180) or (gyroXAngle > 180)):
          gyroXAngle = kalAngleX
      if ((gyroYAngle < -180) or (gyroYAngle > 180)):
          gyroYAngle = kalAngleY

      self.t = kalAngleY
    #self.t = self.controller.get_current_angle()[1]
    #self.dt = self.controller.get_current_angle_speed()
    
      return self.t

  def do_action(self, action):
        if action==True:
            GPIO.output(in1,GPIO.HIGH)
            GPIO.output(in2,GPIO.LOW)
            GPIO.output(in3,GPIO.LOW)
            GPIO.output(in4,GPIO.HIGH)
            p.ChangeDutyCycle(100)
            q.ChangeDutyCycle(100)
        else:
            GPIO.output(in2,GPIO.HIGH)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in4,GPIO.LOW)
            GPIO.output(in3,GPIO.HIGH)
            p.ChangeDutyCycle(100)
            q.ChangeDutyCycle(100)
        
      
  def update_all_weights(self, rhat, failed):
    for i in range(self.n_states):
        self.action_weights[i] += self.alpha * rhat * self.action_weights_elig[i]
        self.critic_weights[i] += self.beta * rhat * self.critic_weights_elig[i]

        if self.critic_weights[i] < -1.0:
          self.critic_weights[i] = self.critic_weights[i]

        if failed == True:
          self.action_weights_elig[i] = 0
          self.critic_weights_elig[i] = 0
        else:
          self.action_weights_elig[i] = self.action_weights_elig[i] * self.lambda_w
          self.critic_weights_elig[i] = self.critic_weights_elig[i] * self.lambda_v

        
          
    