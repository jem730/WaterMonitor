# WaterMonitor
Description: This project aims to develop a flexible monitoring system with two distinct applications. The primary objective is to create a water tank level monitoring system for a 1000-gallon tank. The proposed setup will use an ESP32 V3 with LoRa for wireless data transmission and an RCWL-1670 ultrasonic rangefinder for depth measurements, transmitting data to a NUCLEO microcontroller with an OLED display. The system is designed to update every 10 minutes or so, providing reliable information for water usage tracking while conserving power.
A secondary goal is to adapt the system for tide and wave tracking, enabling it to monitor surf conditions by capturing data on tides and wave activity. This configuration would involve higher-frequency measurements, and potentially some hardware modifications to allow the system to deliver real-time insights on tidal changes and wave patterns. By designing with both applications in mind, this project intends to produce a modular solution adaptable to both water level monitoring and surf condition analysis.

![image](https://github.com/user-attachments/assets/d556eb2f-8c26-49ae-8807-d0ea97859ae5)

![image](https://github.com/user-attachments/assets/cc3e18db-c270-4d96-98d3-85bc43db3a98)



ESP32-S3FN8 MCU Datasheet:
https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf

ESP32-S3 MCU Reference Manual:
https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf

Heltec Development Board Datasheet: 
https://resource.heltec.cn/download/WiFi_LoRa_32_V3/HTIT-WB32LA_V3(Rev1.1).pdf

Built-in OLED Display Font Generator: 
https://oleddisplay.squix.ch/
