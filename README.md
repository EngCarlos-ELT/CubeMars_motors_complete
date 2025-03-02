This application provides a comprehensive graphical interface for controlling and monitoring a CubeMars AK80-64 brushless motor through the CAN (Controller Area Network) protocol. The software is designed with cross-platform compatibility, automatically detecting whether it's running on Windows or Linux and configuring the appropriate CAN interface. Users can precisely control the motor's position, velocity, and torque while adjusting control gains (Kp and Kd) through an intuitive slider-based interface. The application supports both single commands and continuous control with adjustable update rates, allowing for smooth motion profiles and real-time adjustments. Motor feedback is continuously displayed, showing the actual position, velocity, and torque values as reported by the motor's internal sensors. 

Special commands for entering MIT (Model Independent Trajectory) mode, exiting control modes, and zeroing the position reference are accessible through dedicated buttons. The modular code structure separates motor communication, UI components, and application logic into distinct files, making the system maintainable and extensible for future enhancements or adaptation to other motor models. This tool is particularly valuable for robotics applications, motion control systems, and educational settings where precise motor control and monitoring are required.

IMPORTANT!
To run this code, the following dependencies are necessary:
pip install python-can
pip install pyserial
pip install kivy


File Structure Overview.

constants.py: This file centralizes all the motor parameters and operational limits in one place, including position limits (-12.5 to 12.5 radians), velocity limits (-8.0 to 8.0 rad/s), gain limits for position and velocity control, and torque limits (-144.0 to 144.0 N·m). By isolating these constants, the code becomes more maintainable as any adjustments to motor specifications only need to be changed in this single file rather than throughout the codebase.

motor.py: The motor module handles all communication with the CubeMars AK80-64 motor, containing the MotorState class that tracks both commanded and measured values, conversion functions for translating between floating-point physical units and integer CAN protocol values, and command functions for entering/exiting MIT mode, zeroing position, sending control commands, and decoding motor responses. It also includes platform detection to automatically configure the appropriate CAN interface for Windows or Linux systems.

ui_components.py: This file defines reusable custom Kivy UI components, including the LabeledSlider class that combines a label, slider, and value display in a vertical layout, and the StatusDisplay class that presents the motor's current position, velocity, and torque readings in a clean grid layout. These components encapsulate their internal logic and presentation, making the main application code cleaner and more focused on application logic rather than UI details.

app.py: The application file contains the MotorControlApp class that serves as the central controller, building the user interface with sliders for position, velocity, gains, and torque control, along with connection management, command buttons, and status display. It handles all user interactions, manages the connection to the motor via CAN, processes continuous control updates, and maintains a background thread for receiving motor status updates while keeping the UI responsive.

main.py: This simple entry point file imports the MotorControlApp class and runs the application, serving as the starting point for the program and keeping the initialization code separate from the application logic. It's a clean, minimal file that follows the standard pattern for Kivy applications and makes it clear where execution begins.





Here's a summary of the complete flow for adding new UI elements:

Define Custom UI Components (if needed)
    Add custom widget classes to ui_components.py
    Extend existing Kivy widgets with additional functionality
Add UI Elements to the App
    Modify the build() method in app.py
    Create new widgets and add them to the layout
    Bind event handlers to the widgets
Implement Event Handlers
    Add new methods to the MotorControlApp class
    Implement the functionality for the new UI elements
Update Unit Tests
    Add tests for the new functionality in unit_tests.py
    Mock UI components and test the event handlers
Update Integration Tests
    Add tests for the new functionality in integration_tests.py
    Test the interaction between UI elements and the application logic



