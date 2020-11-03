from picar import PiCar, test, configure
import sys
import time        
from time import sleep
import matplotlib
matplotlib.use('Pdf')
import matplotlib.pyplot as plt
import numpy as np

duration = 12
sampledelay=.005
delayspeed=.1
RPS=5
DEBUG = True
Gain = 15

#Parameters used for PID control
Kp= Gain/8
Ki=.1
Kd=.1

errorStore = 0
errorSum = 0
carStatus = False
newpwm=45
if len(sys.argv)>1:
    newpwm=float(sys.argv[1])
if len(sys.argv)>2:
    RPS = float(sys.argv[2])
if len(sys.argv)>3:
    delayspeed=float(sys.argv[3])
if len(sys.argv)>4:
    sampledelay = float(sys.argv[4])
if len(sys.argv)>7:
    if(int(sys.argv[7])>0):
        DEBUG = True
if len(sys.argv)>5:
    Kp = float(sys.argv[5])
if len(sys.argv)>6:
    Ki = float(sys.argv[6])

car = PiCar(mock_car=carStatus, threaded=True)

PWM = RPS*Gain
if PWM>100:
    PWM=100
if PWM<0:
    PWM=0
MAXSIZE = 10000
data=[0]*MAXSIZE
trans=[0]*MAXSIZE
speed=[0]*MAXSIZE
deriv = [0]*MAXSIZE


port=0

data[0]=car.adc.read_adc(port)
deriv[0]=0

thresh = (1023-data[0])*.2

speed=0
i=1
j=0
k=0


t_start = time.time()
t_lastreading = t_start
t_lastcalc = t_start
t_now = t_start

car.set_motor(PWM)

switch = True

with open("file.txt","w") as datafile:

    #Main while loop
    while(t_now-t_start<duration):
        t_now=time.time()
        if(t_now-t_lastreading>sampledelay):
            data[i] = car.adc.read_adc(port) #number of adc used
            deriv[i] = data[i]-data[i-1]
            if switch:
                if deriv[i]>thresh:
                    trans[i]=1
                    switch = False
            else:
                if deriv[i]<-thresh:
                    trans[i]=-1
                    switch = True
            datafile.write(f'{t_now-t_start:.2f}\t {data[i]:.2f}\t {speed:.2f}\n')
            t_lastreading=t_now
            z=i
            i = (i+1)%MAXSIZE
        if(t_now-t_lastcalc>delayspeed):
            if DEBUG:
                print(f'Time: {t_now-t_start:.2f}\t Data: {data[z]:.2f}\t Speed: {speed:.2f}\t PWM: {PWM}\t Tresh: {thresh}\n')
            transCt = 0
            lookback = 100
            if lookback>z:
                lookback=z
            period = lookback*sampledelay
            thresh = .2*max(deriv[z-lookback:z])
            if thresh<20:
                thresh=20

            #Calculates RPM
            for dat in trans[z-lookback:z]:
                if dat>0:
                    transCt = transCt+1
                if dat<0:
                    transCt = transCt+1
            speed = transCt/(period*4)
            error = RPS - speed
            errorSum = errorSum + error
            errorDiff = error - errorStore
            controlKi = errorSum*Ki
            controlKd = errorDiff*Kd
            controlKp = error*Kp
            if DEBUG:
                print(f'Control-I: {controlKi}\t Control-D: {controlKd}\t Control-P: {controlKp}\n')
            PWM = (controlKp + controlKd + controlKi)+PWM

            if PWM>100:
                PWM=100
                print('maxed out PWM')
            if PWM<0:
                PWM=0
                print('PWM under 0')

            #Resets PWM after .35 second delay
            if t_now-t_start>.35:
                car.set_motor(PWM)
            t_lastcalc=t_now
            j=(j+1)%MAXSIZE
            errorStore=error
car.set_motor(0)
car.stop()






file    = open('file.txt', 'r')
info    = file.read().splitlines()  # split lines into an array
MAXSIZE = len(info)

time    = [0]*MAXSIZE
speed   = [0]*MAXSIZE



i=0
for inf in info:
    values   = inf.split()
    time[i]  = float(values[0])
    speed[i] = float(values[2])
    i = i+1

xmarks = np.linspace(time[0], time[MAXSIZE - 1], 5)
plt.xticks(xmarks)

plt.plot(time, speed)
plt.grid()
plt.xlabel('time - sec')
plt.ylabel('RPS')
plt.savefig(f'speed_nick.png')

plt.clf()

#Plotting Transitions
plt.plot(trans[0:int(MAXSIZE/2)])
plt.savefig('transitions.png')
plt.clf()

#Plotting Derivative
plt.plot(deriv[0:int(MAXSIZE/2)])
plt.savefig('deriv.png')
plt.clf()

#Plotting Data
plt.plot(data[0:int(MAXSIZE/2)])
plt.savefig('data.png')
