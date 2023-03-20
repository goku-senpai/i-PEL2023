# MOTOR COntrol i-PEL
## CUBE MX Setup for the peripherals:
    Setting up CubeMX for C++:
        Open CubeMX and create a new project for your microcontroller.
        In the Project Settings tab, select "C++" as the Language.
        In the Code Generator tab, make sure "Copy only necessary files" is selected.
        Click on the "Generate Code" button to generate the project files.

    Configuring the Pinout and Peripherals:
        Click on the "Pinout & Configuration" tab.
        For USB configuration, locate the USB_OTG_FS and USB_OTG_HS peripherals, and enable them.
        For UART configuration, locate the USART peripherals that correspond to the UART pins you want to use, and enable them. Then, assign the appropriate pins in the "Pinout" tab.
        For GPIO configuration, locate the GPIO peripheral that corresponds to the pin you want to use, and enable it. Then, assign the appropriate pin in the "Pinout" tab.
        For motor direction pins, locate the GPIO peripherals that correspond to the pins you want to use, and enable them. Then, assign the appropriate pins in the "Pinout" tab.

    Additional Configuration:
        For PWM configuration, you will need to use a timer peripheral. Locate the timer peripheral that corresponds to the pin you want to use, and enable it. Then, assign the appropriate pin in the "Pinout" tab. You can configure the timer to generate a PWM signal using the timer's PWM mode.
        For encoder input, you will need to use a timer peripheral with an encoder interface. Locate the timer peripheral that corresponds to the UART pins you want to use, and enable it. Then, assign the appropriate pins in the "Pinout" tab. You can configure the timer to use its encoder interface to measure the rotation of the encoder.

## SETUP KF
### how to tune the PARAMETERS:

    One approach to tuning SPEED_KF is to first tune the PID gains for the speed control loop without any feedforward term (i.e., setting SPEED_KF to zero), so that the controller is able to regulate the speed accurately. Once the PID gains are tuned, you can start increasing SPEED_KF gradually and observe the response of the motor to step changes in the setpoint. A higher value of SPEED_KF will cause the motor to respond faster to changes in the setpoint, but may also introduce overshoot and instability. You can experiment with different values of SPEED_KF until you find a value that gives good performance without introducing instability.
    Alternatively, you can use a system identification technique to estimate the motor parameters and tune the controller gains and feedforward term automatically. There are several methods for system identification, such as the least-squares method or the recursive least-squares method, that can estimate the motor parameters from input-output data collected during system operation. Once the parameters are estimated, you can use optimization techniques to tune the controller gains and feedforward term to meet your performance objectives.

### Startingpoint:
    As a starting point, a typical value for SPEED_KF in a closed-loop motor control system with an encoder-based feedback could be between 0.1 and 0.5. However, this value could be different depending on the specific system requirements and the motor's characteristics, so it's best to use this as a starting point and then tune the value experimentally to achieve the desired performance.

## Calculate PID:
### DC-Motor
    To calculate the PID gains for a DC motor, you need to measure the motor's speed and current while it's running under load. Then, you can use the following formula:

        L = Kt / R 

    Where L is the PID gain, Kt is the torque constant, and R is the armature resistance.

### Stepper Motor
    To calculate the PID gains for a stepper motor, you need to measure the motor's torque and speed while it's running under load. Then, you can use the following formula:
    
        L = (Kt / R) * (60 / S)
    
    Where L is the PID gain, Kt is the torque constant, R is the winding resistance, and S is the stepping rate in steps per second.

### Servo Motor:
    To calculate the PID gains for a servo motor, you need to measure the motor's velocity and torque while it's running under load. Then, you can use the following formula:
    
        L = Kt / Kb
    
    Where L is the PID gain, Kt is the torque constant, and Kb is the back-emf constant.


