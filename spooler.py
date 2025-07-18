"""File to control the spooling process"""
import time
import math
import numpy as np
import RPi.GPIO as GPIO
from gpiozero import RotaryEncoder

from database import Database
from user_interface import UserInterface

def sign (value):    #function to get the sign function of a value
        if value == 0:
           res = 0
        if value < 0:
           res = -1
        if value > 0:
           res = 1
        return res   


class Spooler:
    """DC Motor Controller for the spooling process"""
    ENCODER_A_PIN = 24
    ENCODER_B_PIN = 23
    PWM_PIN = 5

    PULSES_PER_REVOLUTION = 1176
    READINGS_TO_AVERAGE = 10
    SAMPLE_TIME = 0.1
    DIAMETER_PREFORM = 7
    DIAMETER_SPOOL = 15.2

    def __init__(self, gui: UserInterface) -> None:
        self.gui = gui
        self.encoder = None
        self.pwm = None
        self.slope = Database.get_calibration_data("motor_slope")
        self.intercept = Database.get_calibration_data("motor_intercept")
        self.motor_calibration = True
        if self.slope == -1 or self.intercept == -1:
            self.motor_calibration = False
        GPIO.setup(Spooler.PWM_PIN, GPIO.OUT)
        self.initialize_encoder()
        
        # Control parameters
        self.previous_time = 0.0
        self.integral_diameter = 0.0
        self.previous_error_diameter = 0.0
        self.previous_steps = 0
        self.integral_motor = 0.0
        self.previous_error_motor = 0.0

        self.previous_rpm = 0.0   #David added this line

    def initialize_encoder(self) -> None:
        """Initialize the encoder and SPI"""
        self.encoder = RotaryEncoder(Spooler.ENCODER_A_PIN,
                                     Spooler.ENCODER_B_PIN, max_steps=0)

    def start(self, frequency: float, duty_cycle: float) -> None:
        """Start the DC Motor PWM"""
        self.pwm = GPIO.PWM(Spooler.PWM_PIN, frequency)
        self.pwm.start(duty_cycle)

    def stop(self) -> None:
        """Stop the DC Motor PWM"""
        if self.pwm:
            self.pwm.stop()

    def update_duty_cycle(self, duty_cycle: float) -> None:
        """Update the DC Motor PWM duty cycle"""
        self.pwm.ChangeDutyCycle(duty_cycle)

    def get_average_diameter(self) -> float:
        """Get the average diameter of the fiber"""
        if len(Database.diameter_readings) < Spooler.READINGS_TO_AVERAGE:
            return (sum(Database.diameter_readings) /
                    len(Database.diameter_readings))
        else:
            return (sum(Database.diameter_readings[-Spooler.READINGS_TO_AVERAGE:])
                    / Spooler.READINGS_TO_AVERAGE)

    def diameter_to_rpm(self, diameter: float) -> float:
        """Convert the fiber diameter to RPM of the spooling motor"""
        stepper_rpm = self.gui.extrusion_motor_speed.value()
        return 25/28 * 11 * stepper_rpm * (Spooler.DIAMETER_PREFORM**2 /
                                        (Spooler.DIAMETER_SPOOL * diameter**2))

    def rpm_to_duty_cycle(self, rpm: float) -> float:
        """Convert the RPM to duty cycle"""
        return self.slope * rpm + self.intercept  

    def motor_control_loop(self, current_time: float) -> None:
        """Closed loop control of the DC motor for desired diameter"""

        flag_u = 0

        if current_time - self.previous_time <= Spooler.SAMPLE_TIME:
            return
        try:
            if not self.motor_calibration:
                self.gui.show_message("Motor calibration data not found",
                                    "Please calibrate the motor.")
                self.motor_calibration = True
            target_diameter = self.gui.target_diameter.value()
            current_diameter = self.get_average_diameter()

            diameter_ku = self.gui.diameter_gain.value()
            diameter_tu = self.gui.diameter_oscilation_period.value()
            diameter_kp = 0.6 * diameter_ku
            diameter_ti = diameter_tu / 2
            diameter_td = diameter_tu / 8
            diameter_ki = diameter_kp / diameter_ti
            diameter_kd = diameter_kp * diameter_td

            motor_ku = self.gui.motor_gain.value()
            motor_tu = self.gui.motor_oscilation_period.value()
            motor_kp = 0.3 #0.3 #0.6 * motor_ku
            motor_ti = motor_tu / 2
            motor_td = motor_tu / 8
            motor_ki = 0.20 #0.35 #motor_kp / motor_ti
            motor_kd = 0.05 #motor_kp * motor_td



            delta_time = current_time - self.previous_time
            self.previous_time = current_time
            error = target_diameter - current_diameter
            self.integral_diameter += error * delta_time
            self.integral_diameter = max(min(self.integral_diameter, 0.5), -0.5)
            derivative = (error - self.previous_error_diameter) / delta_time
            self.previous_error_diameter = error
            output = (diameter_kp * error + diameter_ki * self.integral_diameter
                      + diameter_kd * derivative)
            setpoint_rpm = self.diameter_to_rpm(target_diameter)
            setpoint_rpm = 35 #max(min(setpoint_rpm, 0), 60)

            ### PID control for the motor
            delta_steps = self.encoder.steps - self.previous_steps
            self.previous_steps = self.encoder.steps
            current_rpm = (delta_steps / Spooler.PULSES_PER_REVOLUTION * 
                           60 / delta_time)
            error = setpoint_rpm - current_rpm
            self.integral_motor += error * delta_time
            #self.integral_motor = max(min(self.integral_motor, 100), -100)  #original line
            #self.integral_motor = max(min(self.integral_motor, -100), 100)
            derivative = (error - self.previous_error_motor) / delta_time  #(original line)
            #derivative = (current_rpm - self.previous_rpm) / delta_time    #added by David
            self.previous_error_motor = error
            self.previous_rpm = current_rpm
            output =  (motor_kp * error + motor_ki * self.integral_motor +
                        motor_kd * derivative)
            
            ### Super twisting Sliding modes for the motor

            gain1 = 1   #gain for the sliding surface
            alpha1 = -10  #alpha gain one for the controller
            alpha2 = 10  #alpha gain two for the controller
            h = 0.01    #integration step

            if flag_u == 0:
                u = 0
                flag_u = 1
            
            surface = gain1 * error
            du = -alpha2 * sign(surface)
            u = u + (du * h)

            output = -alpha1 * math.sqrt( abs(surface) ) * sign(surface) + u
            #output = 20


            output_duty_cycle = self.rpm_to_duty_cycle(output) 
            #output_duty_cycle = max(min(output_duty_cycle, 100), 0) # original line
            output_duty_cycle = max(min(output_duty_cycle, 60), 0)   #limited the output for testing
            self.update_duty_cycle(output_duty_cycle)

            # Update plots
            self.gui.motor_plot.update_plot(current_time, current_rpm,
                                            setpoint_rpm)
            self.gui.diameter_plot.update_plot(current_time, current_diameter,
                                                  target_diameter)

            # Add data to the database
            Database.spooler_delta_time.append(delta_time)
            Database.spooler_setpoint.append(setpoint_rpm)
            Database.spooler_rpm.append(current_rpm)
            Database.spooler_gain.append(diameter_ku)
            Database.spooler_oscilation_period.append(diameter_tu)
        except Exception as e:
            print(f"Error in motor control loop: {e}")
            self.gui.show_message("Error", "Error in motor control loop",
                                  "Please restart the program.")

    def calibrate(self) -> None:
        """Calibrate the DC Motor"""
        rpm_values = []
        duty_cycles = []
        num_samples = 5

        try:
            for duty_cycle in range(20, 101, 10):  # Sweep duty cycle from 0% to 100% in increments of 10%
                rpm_samples = []
                for _ in range(num_samples):
                    self.update_duty_cycle(duty_cycle)
                    time.sleep(2)
                    # Measure RPM
                    oldtime = time.perf_counter()
                    oldpos = self.encoder.steps
                    time.sleep(Spooler.SAMPLE_TIME)
                    newtime = time.perf_counter()
                    newpos = self.encoder.steps
                    dt = newtime - oldtime
                    ds = newpos - oldpos
                    rpm = ds / Spooler.PULSES_PER_REVOLUTION / dt * 60
                    rpm_samples.append(rpm)
                avg_rpm = sum(rpm_samples) / num_samples
                duty_cycles.append(duty_cycle)
                rpm_values.append(avg_rpm)
                print(f"Duty Cycle: {duty_cycle}% -> Avg RPM: {avg_rpm:.2f}")

            # Fit a curve to the data
            coefficients = np.polyfit(rpm_values, duty_cycles, 1)
            self.slope = coefficients[0]
            self.intercept = coefficients[1]
            Database.update_calibration_data("motor_slope", str(self.slope))
            Database.update_calibration_data("motor_intercept", str(self.intercept))

        except KeyboardInterrupt:
            print("\nData collection stopped\n\n")

        self.gui.show_message("Motor calibration completed.",
                               "Please restart the program.")
        self.stop()
        self.previous_steps = self.encoder.steps
        print("aaaa")
