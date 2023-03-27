Hello! I'll do my best to help you set up a CubeMX project for your STM32F767ZI Nucleo board.

First, let's start by opening CubeMX and creating a new project. In the "Select a Board" window, choose "NUCLEO-F767ZI" from the list of boards.

For USB communication enable DMA requests for both USART2_RX and USART2_TX.

To do this in CubeMX, follow these steps:

    In the "Pinout & Configuration" tab, click on the "USART2" peripheral to configure it.
    Under the "Mode" section, select "Asynchronous".
    Under the "Parameter Settings" section, set the "Baud Rate" to "115200".
    Under the "Hardware Flow Control" section, set "CTS/RTS" to "None".
    Under the "Parameter Settings" section, enable "DMA".
    Under the "DMA Settings" section, select "USART2_RX" from the "Request Mapping" dropdown menu and enable the DMA channel you want to use for receiving data.
    Similarly, select "USART2_TX" from the "Request Mapping" dropdown menu and enable the DMA channel you want to use for transmitting data.
    Click on "OK" to save the configuration.

Once you have done this, CubeMX will generate the necessary initialization code to enable DMA requests for USART2_RX and USART2_TX. You can then configure and use the DMA channels in your code to handle incoming and outgoing data.

Note that the specific DMA channels and settings you use may depend on your specific use case and requirements. You may need to adjust the settings based on your application's needs.

Next, we need to configure the GPIO pins for the motor encoder. Let's assume you are using a quadrature encoder with two output channels (A and B) and that you are connecting the A and B channels to pins PA0 and PA1, respectively. To configure these pins, follow these steps:

    In the "Pinout & Configuration" tab, click on pins PA0 and PA1 to configure them.
    Under the "Mode" section, select "GPIO_Input".
    Under the "Parameter Settings" section, set "GPIO_Speed" to "Very High".
    Click on "OK" to save the configuration.

Finally, we need to configure the ENA/ENB pins for PWM control of the motor. Let's assume you are using pins PB1 and PB2 for this purpose. To configure these pins, follow these steps:

    In the "Pinout & Configuration" tab, click on pins PB0 and PB1 to configure them.
    Under the "Mode" section, select "PWM Generation CH1" for PB0 and "PWM Generation CH2" for PB1.
    Under the "Parameter Settings" section, set "PWM Frequency" to your desired frequency.
    Click on "OK" to save the configuration.

That's it! Now you can generate the code for your project and start programming. Remember to initialize the USART2 peripheral and configure the GPIO pins for the motor encoder and PWM in your code.