# Thermal Tracking Device #
* This code uses an ESP32 mini, a Panasonic Grid-EYE sensor, SG90 micro servo motors, and several electrical components in order to direct air towards a person


* Tracking Device Processing is code to be used in the Processing Application to display the data from the GridEYE
  * This requires serial data so data must be printed in the serial monitor each loop
  * Sometimes the processing code will not upload due to a NaN error. Just keep trying to upload it (maybe 3-5 times) and it should upload succesfully


* Epsilon is the main version. This was designed to work with an ESP32 mini microcontroller. The pinout is seen below.

![ESP32 Pinout](https://github.com/leo-b13/Thermal-Tracking-Device/blob/main/ESP32_Pinout.png?raw=true)
