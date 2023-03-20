The NUCLEO-144 STM32F767ZI board also has two USB connectors: one is the USB ST-LINK/V2-1 connector (CN1), and the other is the USB device connector (CN12).

For UART communication over the USB device connector, you can use the same pins as for the NUCLEO-F767ZI board:

    PA11 (USB_OTG_FS_DM)
    PA12 (USB_OTG_FS_DP)

So the code snippet I provided earlier should work for both boards without modification, assuming you are using the USB device connector (CN12) for UART communication.

Note that if you are using the USB ST-LINK/V2-1 connector (CN1) for UART communication, you will need to use different pins, as the ST-LINK/V2-1 interface does not support UART communication.