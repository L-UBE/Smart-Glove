# Smart-Glove

ioc file: lines out a lot of changes from the original PCB design. turned off a lot of pins and made them GND for twisted pair wirings. 

VL6180X.h .c : functions for the distance sensor VL6180X
bno055.h .c : functions for the IMU

main.c : 
- full implementation of the state machine with state transition and state performance
- most of the parameters are encoded with "define"'s for easier control

send2App(gesture, data, buffer): 
- function that sends out the gesture and getureData to the app side

