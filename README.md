# flight-attitude-estimation
Prototype of attitude estimation using MPU6050 accelerometer and gyroscope with Arduino UNO

Module ::: mpu_test
Data from the MPU6050 sensor will be sent to microcontroller via I2C protocol
and then microcontroller will calculate the attitude in Local-Vertical Local-Horizontal frame (NED).
Finally, the microcontroller will transmit the data to PC/OBC by UART protocol.

To visualize the attitude represent in Euler angle with Yaw-Pitch-Roll sequence by execute "python main.py"
