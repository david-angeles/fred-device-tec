###############################################
########## declarations of functions ##########

def motor_speed (current_time, previous_time, previous_steps, encoder_steps):
    rpm = ((encoder_steps - previous_steps) * 60) / ((current_time - previous_time) * 1176)
    #previous_steps = encoder.steps
    return rpm 

def save_data (time_data, rpm_data, motor_voltage_data, PWM_motor_data):
    with open("FrED_data.txt","a") as archivo:
        archivo.write("time\trpm\tvolt\tpwm\n")
        size = len(time_data)
        for i in range(size-1):
            a = time_data[i]
            b = rpm_data[i]
            c = motor_voltage_data[i]
            d = PWM_motor_data[i]
            archivo.write(f"{a}\t{b}\t{c}\t{d}\n")

def least_square (current_time):
        ########TEST TO IDENTIFICATION SYSTEM #############
        if current_time < 30:
            output = 25
        if current_time < 60 and current_time >= 30:
            output = 35
        if current_time < 90 and current_time >= 60:
            output = 50
        if current_time < 120 and current_time >= 90:
            output = 70     
        if current_time < 150 and current_time >= 120:
            output = 90 
        if current_time < 180 and current_time >= 150:
            output = 100  
        if current_time < 260 and current_time >= 180:
            output = 100-current_time+180
        if current_time >= 260:
            output = current_time-250
        if current_time >= 340:
            output = 100 
        ####### TEST 2 to IDENTIFICATION SYSTEM ###########
        #if current_time > 30:
        #    output = 120
        #else :
        #    output = 0 
        
        return output

def ploting (time_data):
    # Update the plot
    line1.set_xdata(time_data)
    line1.set_ydata(rpm_data)
    line2.set_xdata(time_data)
    line2.set_ydata(rpm_ref_data)
    ax.relim()
    ax.autoscale_view()
    plt.draw()
    plt.pause(0.01)

def plotD ():
    plt.plot(time_data, rpm_data)
    plt.show()