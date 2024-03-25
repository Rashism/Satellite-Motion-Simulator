import numpy as np
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import unittest
from unittest.mock import Mock

# Define the PID class
class PID:
    
    """
    This class implements a PID controller.

    Parameters
    ----------
    Kp: float
        Proportional gain.
    Ki: float
        Integral gain.
    Kd: float
        Derivative gain.

    Attributes
    ----------
    integral: float
        Accumulated error.
    last_error: float
        Previous error.

    Methods
    -------
    update(error, dt):
        Update the controller with a new error and time step.

    """
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.last_error = 0
        
    """
        Update the controller with a new error and time step.

        Parameters
        ----------
        error: float
            Error between the current and set point.
        dt: float
            Time step.

        Returns
        -------
        control: float
            Control signal.

        """

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

# Define the SatelliteSimulator class
class SatelliteSimulator(QWidget):
    def __init__(self):
        super().__init__()

        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # Create labels and line edits for PID parameters
        kp_label = QLabel("Kp:")
        self.kp_entry = QLineEdit()

        ki_label = QLabel("Ki:")
        self.ki_entry = QLineEdit()

        kd_label = QLabel("Kd:")
        self.kd_entry = QLineEdit()

        setpoint_label = QLabel("Set Point:")
        self.setpoint_entry = QLineEdit()

        sim_time_label = QLabel("Simulation Time:")
        self.sim_time_entry = QLineEdit()

        layout.addWidget(kp_label)
        layout.addWidget(self.kp_entry)
        layout.addWidget(ki_label)
        layout.addWidget(self.ki_entry)
        layout.addWidget(kd_label)
        layout.addWidget(self.kd_entry)
        layout.addWidget(setpoint_label)
        layout.addWidget(self.setpoint_entry)
        layout.addWidget(sim_time_label)
        layout.addWidget(self.sim_time_entry)

        # Create a button to start simulation
        start_button = QPushButton("Start Simulation")
        start_button.clicked.connect(self.start_simulation)
        layout.addWidget(start_button)

        # Create a figure and add it to the layout
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

        self.setLayout(layout)

    def start_simulation(self):
        # Get PID parameters from line edits
        Kp = float(self.kp_entry.text())
        Ki = float(self.ki_entry.text())
        Kd = float(self.kd_entry.text())

        # Get set point and simulation time
        setpoint = float(self.setpoint_entry.text())
        sim_time = float(self.sim_time_entry.text())

        # Run simulation
        time, position, velocity = simulate_satellite_motion(Kp, Ki, Kd, setpoint, sim_time)

        # Plot position vs. time
        ax1 = self.figure.add_subplot(211)
        ax1.plot(time, position)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Position')
        ax1.set_title('Satellite Position vs. Time')

        # Plot velocity vs. time
        ax2 = self.figure.add_subplot(212)
        ax2.plot(time, velocity)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity')
        ax2.set_title('Satellite Velocity vs. Time')

        # Update canvas
        self.canvas.draw()

# Define the unit test class
class TestSatelliteSimulator(unittest.TestCase):
    def test_simulate_satellite_motion(self):
        # Define test parameters
        Kp = 1
        Ki = 2
        Kd = 3
        setpoint = 4
        sim_time = 5

        # Run the simulation
        time, position, velocity = simulate_satellite_motion(Kp, Ki, Kd, setpoint, sim_time)

        # Verify the results
        self.assertEqual(time.shape, (51,))
        self.assertEqual(position.shape, (51,))
        self.assertEqual(velocity.shape, (51,))
        self.assertAlmostEqual(position[-1], 4.0, places=1)
        self.assertAlmostEqual(velocity[-1], 5.0, places=1)
        
        unittest.main()

# Define the function for simulating satellite motion
def simulate_satellite_motion(Kp, Ki, Kd, setpoint, sim_time):
    pid = PID(Kp, Ki, Kd)
    time = np.arange(0, sim_time, 0.1)
    position = np.zeros_like(time)
    velocity = np.zeros_like(time)

    for i in range(1, len(time)):
        dt = time[i] - time[i - 1]
        error = setpoint - position[i - 1]
        control = pid.update(error, dt)

        acceleration = control
        velocity[i] = velocity[i - 1] + acceleration * dt
        position[i] = position[i - 1] + velocity[i] * dt

    return time, position, velocity

# Main execution block
if __name__ == "__main__":
    app = QApplication([])
    window = SatelliteSimulator()
    window.setWindowTitle("Satellite Motion Simulator")
    window.show()
    app.exec_()


