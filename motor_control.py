######### Program to control the FrED's DC motor ########



import os
import cv2
import time
import math
import board
import busio
import atexit
import datetime
import digitalio
import contextlib
import numpy as np
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import adafruit_mcp3xxx.mcp3008 as MCP
GPIO.setmode(GPIO.BCM)

from time import sleep
#from gpiozero import Motor
from gpiozero import RotaryEncoder
from adafruit_mcp3xxx.analog_in import AnalogIn


########## GPIO Pin Definitions ##########
encoder_1 = 24         # DC motor encoder input pin 1
encoder_2 = 23         # DC motor encoder input pin 2
dcmotorPin = 5         # DC motor PWM output pin

########## Initialise GPIO ##########
GPIO.setwarnings(False)
GPIO.setup(dcmotorPin, GPIO.OUT)

########## Initialise DC motor encoder ##########
encoder = RotaryEncoder(encoder_1, encoder_2, max_steps=0)

# Create the SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Create the cs (chip select)
cs = digitalio.DigitalInOut(board.D8)

# Create the mcp object
mcp = MCP.MCP3008(spi, cs)

# Create analog inputs connected to the input pins on the MCP3008.
channel_0 = AnalogIn(mcp, MCP.P0)

# Capture video from the camera
#cap = cv2.VideoCapture(0)  # Use the appropriate camera index (0 or 1) if you have multiple cameras connected

# DC Motor initialisation
ppr = 300.8       # Pulses Per Revolution of the encoder
dcFreq = 1000     #DC motor PWM frequency
fanFreq = 1000    #Fan PWM frequency

oldtime = 0                    #old DC time
oldpos = 0                     #old DC position
lasttime = 0                   #old stepper time
#Start DC motor
motor_output = GPIO.PWM(dcmotorPin, dcFreq)

frame_count = 0

########### Initialing lists ##########
time_data = []
rpm_data = []
rpm_ref_data = []
PWM_motor_data = []
motor_voltage_data = []

plt.ion()
fig, ax = plt.subplots()
line1, = ax.plot(time_data, rpm_data, label='Motor speed (rmp)')
line2, = ax.plot(time_data, rpm_ref_data, label='rpm reference (rpm)')
ax.legend()
ax.set_title('DC Motor without PID')

###############################################
########## declarations of functions ##########

def motor_speed (current_time, previous_time, previous_steps):
    rpm = ((encoder.steps - previous_steps) * 60) / ((current_time - previous_time) * 1176)
    #previous_steps = encoder.steps
    return rpm 

def save_data ():
    with open("FrED_data.txt","a") as archivo:
        archivo.write("time\trpm\t\tvolt\tpwm\n")
        size = len(time_data)
        for i in range(size-1):
            a = time_data[i]
            b = rpm_data[i]
            c = motor_voltage_data[i]
            d = PWM_motor_data[i]
            archivo.write(f"{a}\t{b}\t\t{c}\t{d}\n")

def ploting ():
    # Update the plot
    line1.set_xdata(time_data)
    line1.set_ydata(rpm_data)
    line2.set_xdata(time_data)
    line2.set_ydata(rpm_ref_data)
    ax.relim()
    ax.autoscale_view()
    plt.draw()
    plt.pause(0.01)

########### variables ###########
rpm_reference = 30  # desired motor speed
previous_time = 0
previous_steps = 0
tm = 0.01           # Sample time
match_time = 0.010
PWM_motor = 0       #DC motor PWM
        

muestra = 1

########## starting to count time ##########
tstart = time.perf_counter()   #start internal clock
initial_time = time.time()

try:
    #Loop Execution
    while True:
        current_time = time.perf_counter() - tstart
        rpm = motor_speed (current_time, previous_time, previous_steps) #measure rpm
        previous_time = current_time
        previous_steps = encoder.steps
        

        #dt = time.perf_counter()-oldtime
        #ds = encoder.steps - oldpos
        #rpm = ds/ppr/dt*60
        if current_time>=muestra:
            print("DC Speed = {:0.2f} rpm".format(rpm))
            print("time =",current_time)
            print("pasos =",encoder.steps)
            muestra = muestra + 1
        print("time =",current_time)
        timeD = time.time() - initial_time
        print("timeD =",timeD)

        
        oldtime = time.perf_counter()
        oldpos = encoder.steps

        ####################################################
        ########TEST TO IDENTIFICATION SYSTEM #############
        #if current_time < 30:
        #    output = 25
        #if current_time < 60 and current_time >= 30:
        #    output = 35
        #if current_time < 90 and current_time >= 60:
        #    output = 50
        #if current_time < 120 and current_time >= 90:
        #    output = 70     
        #if current_time < 150 and current_time >= 120:
        #    output = 90 
        #if current_time < 180 and current_time >= 150:
        #    output = 100  
        #if current_time < 260 and current_time >= 180:
        #    output = 100-current_time+180
        #if current_time >= 260:
        #    output = current_time-250
        #if current_time >= 340:
        #    output = 100 
        ###################################################
        ####### IDENTIFICATION SYSTEM 2 ###########
        #if current_time > 30:
        #    output = 120
        #else :
        #    output = 0 

        PWM_motor = 100
        motor_output.start(PWM_motor)   #sending PWM to the motor

        #ret, frame = cap.read()

        #ploting ()

        ########### convertion PWM to votage ##########
        motor_voltage = (12 * PWM_motor) / 100

        ########### save data to the lists #########
        time_data.append(round(current_time, 4))
        rpm_data.append(round(rpm, 4))
        rpm_ref_data.append(rpm_reference)
        PWM_motor_data.append(PWM_motor)
        motor_voltage_data.append(motor_voltage)

        ########## to have a consistent sample time ##########
        wait = max(0, match_time - current_time)
        time.sleep(wait)
        match_time = match_time + tm
except KeyboardInterrupt:
    print ("\nCode Stopped\n")
    ########## save data in a txt file ##########
    save_data ()
    print ("Data saved in FrED_data.txt file\n\n")
    #print (time_data)
    
finally:
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()

