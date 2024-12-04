Dec 04, 2024

This code runs on PickerV3.5 with 4 motors: Shoulder RMDX8 (0x141), Elbow RMDX6 (0x143), Gripper PWM Servo Motor (0x144) and a z axis Teknic Clearpath SDSK 2310.

This arduino is connected to 2 STM32s through GPIO, one for the Cutter and one for Moving Forward/Backwards.

There is a 8 channel relay which controls the vacuum pump for the suction cup, a release valve, the conveyor motors and the ejector motor.
