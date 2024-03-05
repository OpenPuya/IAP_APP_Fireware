================================================================================
                        Sample Usage Instructions
================================================================================
Function description:
This sample demonstrates the IAP upgrade function, with the IAP program running 
at address 0x080000000 and the APP program running at address 0x08001000.
================================================================================
Test environment:
Testing board: PY32L020_ STK
MDK version: 5.28
IAR version: 9.20
================================================================================
Usage steps:
1. Before powering on, do not press the button PA0 and run the IAP program. 
You can use the PY32IspTool tool to upgrade the APP program through USART.
2. Press the button PA0 continuously before powering on, and the MCU will 
directly jump to the APP program to run.
3. USART1_TX(PA3) USART1_RX(PA4)
================================================================================
Precautions:
================================================================================