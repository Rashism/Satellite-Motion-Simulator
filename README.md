#Satellite Motion Simulator

#Overview
The Satellite Motion Simulator is a Python-based application designed to simulate the motion of a satellite using a PID (Proportional-Integral-Derivative) controller. The project aims to provide users with a graphical tool for learning, testing, and visualizing the behavior of PID controllers in controlling dynamic systems, such as satellite motion.

#Features
.Graphical User Interface (GUI): Intuitive interface for user interaction.
.Real-time Simulation: Simulates satellite motion based on user-defined PID parameters.
.Visualization: Displays satellite position and velocity over time.
.Educational Tool: Helps users understand PID control principles through practical experimentation.

#Purpose
The Satellite Motion Simulator serves as an educational tool for students, engineers, and software developers interested in learning about control systems, specifically PID controllers, and their applications in satellite motion control. By providing a user-friendly interface and real-time visualization, the simulator facilitates hands-on exploration and experimentation with PID parameters and their effects on system behavior.

#Target Audience
.Students: Studying control theory, dynamics, or aerospace engineering.
.Engineers: Working on projects involving control systems or satellite technology.
.Developers: Interested in learning about PID control implementation and visualization.

#How to Use:

#Installation
Before running the Satellite Motion Simulator, ensure you have Python installed on your system. You can download Python from here.

#Dependencies
Install the required Python packages using pip:
.pip install numpy
.pip install matplotlib
.pip install PyQt5

#Usage
1.Clone or download the repository to your local machine.
2.Navigate to the project directory in your terminal or command prompt.
3.Run the main.py file using Python:
python main.py
The Satellite Motion Simulator window will appear, allowing you to input PID parameters and start the simulation.

#Example
# Define PID parameters
Kp = 1.0
Ki = 0.5
Kd = 0.2

# Setpoint and simulation time
setpoint = 100.0
sim_time = 20.0


# Run simulation
time, position, velocity = simulate_satellite_motion(Kp, Ki, Kd, setpoint, sim_time)


#Future Enhancements
Support for additional control algorithms.
Advanced visualization options.
Integration with external data sources or APIs for real-world satellite data.

#License
This project is licensed under the MIT License.
