import sys
from typing import Tuple
from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout, QMainWindow
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QLabel, QDoubleSpinBox 
from PyQt5.QtWidgets import QSlider, QPushButton, QMessageBox, QLineEdit, QCheckBox 

from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
#import matplotlib.pyplot as plt

from database import Database
#from extruder import Thermistor

import matplotlib
matplotlib.use('Qt5Agg')

from PyQt5 import QtCore, QtWidgets

class ControlWindow(QWidget):

    def __init__(self, ui_reference) -> None:
        super().__init__()
        self.ui = ui_reference
        #self.timer = QTimer()
        #self.timer.timeout.connect(self.fiber_camera.camera_loop)
        #self.timer.timeout.connect(self.update_graphs)
        #self.timer.start(500)
        #self.app = QApplication([])
        #self.window = QWidget()
        self.layout = QGridLayout()

        #calls to plot the first column
        self.add_plots()
        
        #self.temperature_plot = self.add_plots()
        #self.diameter_plot = self.add_plots()
        #self.temperature_control_plot = self.add_plots()

        #calls to plot the second column
        #self.fan_extrusion_plot = self.add_plots()    #graph fot the fans and extrusion motor
        #self.spooling_plot = self.add_plots()
        #self.spooling_control_plot = self.add_plots()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graphs)
        self.timer.start(500)

        self.setLayout(self.layout)
        self.setWindowTitle("Control Mode")
        self.setGeometry (800, 800, 1550, 800)
        #self.window = QWidget()
        #self.layout = QGridLayout()


        #self.layout = QVBoxLayout()
        #self.setLayout(self.layout)

        #self.figure = plt.figure()
        #self.canvas = FigureCanvas (self.figure)
        #self.layout.addWidget(self.canvas)

        #self.create_plot()

    #def create_plot(self):
    #    x = [1, 2, 3, 4, 5]
    #    y = [1, 2, 3, 4, 5]

    #    self.figure.clear()
    #    ax=self.figure.add_subplot(341)
    #    ax.plot(x, y, 'r-')
    #    ax.set_title('Grafico de ejemplo')
    #    self.canvas.draw()


    def add_plots(self):
        """Add plots to the layout"""
        font_style = "font-size: 16px; font-weight: bold;"
        #self.layout = QGridLayout()
        #binary_checkbox = QCheckBox("Binary")
        #binary_checkbox.setStyleSheet(font_style)
       # #binary_checkbox.stateChanged.connect(checkbox_state_changed)
    
        self.temperature_plot = self.Plot("Temperature", "Temperature (C)", "Terget temperature")
        #diameter_plot = self.Plot("Diameter", "Diameter (mm)", "Terget diameter")
    #    plot, widget = self.Plot("Temperature", "Temperature (C)", "Terget temperature")
    #    self.temperature_plot = plot
    #    self.layout.addWidget(widget, 1, 0)

        self.diameter_plot = self.Plot("Diameter", "Diameter (mm)", "Terget diameter")
        #temperature_control_plot = self.Plot("Temperature control signal", "Control signal (v)", "Control signal")

        #fan_extrusion_plot = self.Plot("Fan and extrusion motor", "Speed (RPM)", "speed")
        #spooling_plot = self.Plot("Spooling Motor", "Speed (RPM)", "Target speed")
        #spooling_control_plot = self.Plot("Spooling motor control signal", "Control signal (PWM)", "Control signal")
        
        #layout.addWidget(widget, row, column, rowSpan, columnSpan, alignment)
        #self.layout.addWidget(binary_checkbox, 10, 1)
        self.layout.addWidget(self.diameter_plot, 0, 0)
        self.layout.addWidget(self.temperature_plot, 1, 0)
        #self.layout.addWidget(temperature_control_plot, 2, 0)

        #self.layout.addWidget(fan_extrusion_plot, 0, 1)
        #self.layout.addWidget(spooling_plot, 1, 1)
        #self.layout.addWidget(spooling_control_plot, 2, 1)

        #return temperature_plot, temperature_control_plot, diameter_plot, fan_extrusion_plot, spooling_plot, spooling_control_plot
        #return temperature_plot, diameter_plot

    class Plot(FigureCanvas):
        """Base class for plots"""
        def __init__(self, title: str, y_label: str, s_label: str) -> None:
            self.figure = Figure()
            #if block == 1:
            self.axes = self.figure.add_subplot(111)
            #if block == 2:
            #    self.axes = self.figure.add_subplot(111)    
            super(ControlWindow.Plot, self).__init__(self.figure)

            self.axes.set_title(title)
            self.axes.set_xlabel("Time (s)")
            self.axes.set_ylabel(y_label)

            self.progress_line, = self.axes.plot([], [], lw=2, label=title)
            self.setpoint_line, = self.axes.plot([], [], lw=2, color='r', label=s_label)
            self.axes.legend()

            self.x_data = []
            self.y_data = []
            self.setpoint_data = []

        def update_plot(self, x: float, y: float, setpoint: float) -> None:
            """Update the plot"""
            self.x_data.append(x)
            self.y_data.append(y)
            self.setpoint_data.append(setpoint)
            
            #Change the legend to show the current value
            self.progress_line.set_label(f"{self.axes.get_title()}: {y:.1f}")
            self.axes.legend()

            self.progress_line.set_data(self.x_data, self.y_data)
            self.setpoint_line.set_data(self.x_data, self.setpoint_data)

            self.axes.relim()
            self.axes.autoscale_view()
            self.draw()


    def update_graphs(self):
        import time
        import fan
        t = time.time()
        value = t % 10
        setpoint = 5.0
        aaa = dir(self.ui)
        print (aaa)
        temp = self.ui.fan_duty_cycle
        #temp_sp = self.ui.temperature_setpoint
        #self.temperature_plot.update_plot(t, temp, 5.0)
        self.diameter_plot.update_plot(t, value + 1, setpoint + 1)
        #self.temperature_control_plot.update_plot(t, value - 1, setpoint - 1)

        #self.fan_extrusion_plot.update_plot(t, value + 2, setpoint + 2)
        #self.spooling_plot.update_plot(t, value - 2, setpoint - 2) 
        #self.spooling_control_plot.update_plot(t, value + 2, setpoint + 2)   
