================================================================================
                        Sample Usage Instructions
================================================================================
Function description:
This example demonstrates the GPIO external interrupt function, 
where each falling edge on the PA0 pin generates an interrupt. 
In the interrupt function the LED light will flip once.
================================================================================
Test environment:
Testing board: PY32L020_ STK
MDK version: 5.28
IAR version: 9.20
================================================================================
Usage steps:
1. Compile and download the program to MCU and run it;
2. Observe the LED flipping once every time the user presses the button;
================================================================================
Precautions:
The pin corresponding to the user button is PA0
================================================================================