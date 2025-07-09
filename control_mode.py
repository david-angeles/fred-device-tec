from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout, QMainWindow
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QLabel, QDoubleSpinBox 
from PyQt5.QtWidgets import QSlider, QPushButton, QMessageBox, QLineEdit, QCheckBox 

from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt


class ControlWindow():        #class window for setting advanced controllers
    """ This is an extra windows for setting advanced control laws"""
    def __init__(self) -> None:
        super().__init__()
        #self.app = QApplication([])
        self.window = QWidget()
        self.layout = QGridLayout()
        self.window.setWindowTitle("Control Mode")   #before Advance Mode
        self.window.setGeometry (800, 800, 1550, 800)
        #self.app = QApplication([])
        #self.window = QWidget()
        #self.layout = QGridLayout()
        
    def add_plots(self):
        """Add plots to the layout"""
        font_style = "font-size: 16px; font-weight: bold;"
    #    binary_checkbox = QCheckBox("Binary")
    #    binary_checkbox.setStyleSheet(font_style)

        #import matplotlib.pyplot as plt
        #x = [1, 2, 3, 4, 5]
        #y = [1, 4, 9, 16, 25]
        #fig = plt.figure()
        #fig.add_subplot(111)
        #plt.scatter(x, y)
        #plt.show()

        diameter_plot = self.Plot("Diameter", "Diameter (mm)")
        self.layout.addWidget(diameter_plot, 2, 0, 8, 4)

    class Plot(FigureCanvas):
        """ Base class for plots """
        def __init__(self, title: str, y_label: str) -> None:
            self.figure = Figure()
            self.axes = self.figure.add_subplot(111)
            # 1x1 grid, first subplot: https://stackoverflow.com/a/46986694
            super(ControlWindow.Plot, self).__init__(self.figure)

            self.axes.set_title(title)
            self.axes.set_xlabel("Time (s)")
            self.axes.set_ylabel(y_label)

            self.progress_line, = self.axes.plot([], [], lw=2, label=title)
            self.setpoint_line, = self.axes.plot([], [], lw=2, color='r',
                                                    label=f'Target {title}')
            self.axes.legend()

            self.x_data = []
            self.y_data = []
            self.setpoint_data = []
        
        def update_plot(self, x: float, y: float, setpoint: float) -> None:
            # Update the plot
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