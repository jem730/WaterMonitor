To program the Heltec ESP32 V3 in the Arduino IDE:

1. Follow the online tutorial to install the Heltec ESP32 framework and libraries in Arduino IDE:
https://docs.heltec.org/en/node/esp32/esp32_general_docs/quick_start.html

2. Install the system USB drivers (device won't be recognized without these):
https://docs.heltec.org/general/establish_serial_connection.html

Note: For windows 11, the "Universal Windows Driver" doesn't have the exe files. Install the "CP210x Windows Drivers" instead.
![image](https://github.com/user-attachments/assets/7cac85d6-442c-4524-b5ba-7d867cf93140)


4. Select the Heltec ESP32 V3 from the boards manager menu.
   ![image](https://github.com/user-attachments/assets/068fe403-0d21-49d1-a929-5c7eb5f1cd45)

5. You can now program the Heltec ESP32 with your own code.

Note:
1. Many of the example files will also require the ADA GFX libary. On the left-hand side of the Arduino IDE interface, click on the libaries icon, search for the ADA GFX library and install.
