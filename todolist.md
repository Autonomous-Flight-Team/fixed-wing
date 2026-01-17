# Colin's to-do list


- ~~Get Arduino_FreeRTOS to work inside the libraries/ folder of the repository for easier compilation on other machines~~ This is impossible? Arduino IDE doesn't seem to support this, solution is to copy library manually into ~/Documents/Arduino
- Optimize the UKF to use less memory (don't use Eigen::Matrix?)
- Install the IMU and Barometer on the breadboard and see if they actually work properly w/ IMUBaroTask()
