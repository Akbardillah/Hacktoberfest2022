import numpy as np
import serial
import time

def get_jacobian(th):
    J1 = np.matrix([[np.cos(th), 0.], [np.sin(th), 0.], [0., 1]])
    J2 = np.matrix([[r/2., r/2.], [r/L, -r/L]])
    return J1*J2

#robot
r = 0.05 #radius of wheels (meter)
L = 0.15 #distance between wheels (meter)
Ts = 0.1 # Time sampling (seconds)
lamda = 40. # gain error /determines motor speed
min_pwm = 0
max_pwm = 1023

x = np.matrix([[1., 1., np.radians(90.)]]).transpose()
x_des = np.matrix([[6., 4., np.radians(120.)]]).transpose()

# 1. Control orientation of robot towards target point
traj = x_des - x
th_traj = np.arctan2(traj[1,0], traj[0,0])

# Set up serial connection to Arduino
ser = serial.Serial('COM6', 9600)
ser.timeout = 1
time.sleep(0.5)
  # wait for Arduino to initialize
  
rpm_left = ser.readline().decode('ascii')
rpm_right = ser.readline().decode('ascii')

# print the rpm values
print(f'RPM left: {rpm_left}, RPM right: {rpm_right}')
    
for i in range(0,1000):
    J = get_jacobian(x[2,0]) #menghitung nilai jacobian
    Jinv = np.linalg.pinv(J) #invers nilai jacobian
    E = x_des - x # menghitung error
    E[0,0] = 1 #x dianggap 0
    E[1,0] = 1 #y dianggap 0
    E[2,0] = 0 #th_traj - x[2,0] 
    if(np.linalg.norm(E) < 0.1):
        break
    
    qdot = Jinv * lamda * E
    print(qdot)
    xdot = J * qdot
    #update posisi
    
    x = x + xdot*Ts
    
    # Set the duty cycle for the left and right motors
    pwm_left = float(qdot[0,0])
    pwm_right = float(qdot[1,0])

       # Ensure that the duty cycle values are within the valid range (0 to 255)
    #pwm_left = max(min(pwm_left, 1023), 0)
    #pwm_right = max(min(pwm_right, 1023), 0)

       # Send the PWM signals to the Arduino
    #ser.write([pwm_left])
    #ser.write([pwm_right])
    #time.sleep(Ts)
    test = str(pwm_left) + "," + str(pwm_right)
    print("mengirim data")
    ser.write(test.encode('utf-8'))
    time.sleep(Ts)
print("pwmL=",(pwm_left))
print("pwmR=",(pwm_right))
    
ser.close()
