###############################################
########## declarations of functions ##########

def motor_speed (current_time, previous_time, previous_steps, encoder_steps):
    rpm_raw = ((encoder_steps - previous_steps) * 60) / ((current_time - previous_time) * 1176)
    #previous_steps = encoder.steps
    return rpm_raw 

def filter (rpm_raw, previous_rpm):
    alpha = 0.3
    rpm = alpha * rpm_raw + (1 - alpha) * previous_rpm
    return rpm 

def save_data (time_data, rpm_data, motor_voltage_data, PWM_motor_data, rpm_raw_data):
    with open("FrED_data.txt","a") as archivo:
        archivo.write("time\trpm\tvolt\tpwm\traw\n")
        size = len(time_data)
        for i in range(size-1):
            a = time_data[i]
            b = rpm_data[i]
            c = motor_voltage_data[i]
            d = PWM_motor_data[i]
            e = rpm_raw_data[i]
            archivo.write(f"{a}\t{b}\t{c}\t{d}\t{e}\n")

def least_square (current_time, opcion_LS):
        match opcion_LS:
            case 1:
                ########TEST 1 TO IDENTIFICATION SYSTEM #############
                if current_time < 10:
                    output = 100
                if current_time < 20 and current_time >= 10:
                    output = 75
                if current_time < 30 and current_time >= 20:
                    output = 50
                if current_time < 40 and current_time >= 30:
                    output = 25     
                if current_time < 70 and current_time >= 40:
                    output = 25-current_time+40 
                if current_time < 180 and current_time >= 70:
                    output = current_time - 70  
                if current_time < 285 and current_time >= 180:
                    output = 100-current_time+180
                if current_time >= 285:
                    output = 100
                 
            case 2:
                ########TEST 2 TO IDENTIFICATION SYSTEM #############
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
            
            case 3:
                ########TEST 3 TO IDENTIFICATION SYSTEM #############
                if current_time < 10:
                    output = 100
                if current_time < 20 and current_time >= 10:
                    output = 75
                if current_time < 30 and current_time >= 20:
                    output = 50
                if current_time < 40 and current_time >= 30:
                    output = 25     
                if current_time < 43 and current_time >= 40:
                    output = 0 
                if current_time >= 43:
                    output = 100  
            
            case 4:
                ########TEST 4 TO IDENTIFICATION SYSTEM #############
                if current_time < 10:
                    output = 100
                if current_time < 120 and current_time >= 10:
                    output = 100-current_time+10
                if current_time >= 120:
                    output = current_time-120 

            case 5:
                ########TEST 5 TO IDENTIFICATION SYSTEM #############
                if current_time < 10:
                    output = 100
                if current_time < 20 and current_time >= 10:
                    output = 75
                if current_time < 30 and current_time >= 20:
                    output = 50
                if current_time < 40 and current_time >= 30:
                    output = 75
                if current_time >= 40:
                    output = 100 
            
            case 6:         
                ####### TEST 6 to IDENTIFICATION SYSTEM ###########
                if current_time > 0:
                    output = 100
                else :
                    output = 0 
        
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