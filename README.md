# Indoor Environment Monitoring System

## Purpose

This project aims to create an indoor environment monitoring system that tracks temperature, humidity, and light intensity. The system provides real-time data visualization and alerts to enhance comfort and energy efficiency in indoor spaces.

## Features

The Indoor Environment Monitoring System includes the following features:

1. **Environmental Data Collection:** Collect temperature, humidity, and light intensity data using specialized sensors.
2. **Wireless Connectivity:** Connect to WiFi networks for data transmission.
3. **Cloud Integration:** Send sensor data to Thingsboard server via MQTT protocol.
4. **Data Visualization:** Display collected data on a Thingsboard dashboard for easy monitoring.
5. **Alert System:** Set up rule chains in Thingsboard to send alerts to users' Telegram bots when environmental conditions exceed predefined thresholds.

## Required Modules

| Module         | Functionality                                                  |
|----------------|----------------------------------------------------------------|
| ESP32-wroom-32 | Main microcontroller for data processing and WiFi connectivity |
| SHT31          | Temperature and humidity sensor                                |
| BH1750         | Light intensity sensor                                         |

## Detailed Features

1. **Environmental Data Collection**
   
| ID | Functionality              | Note                                          |
|----|----------------------------|-----------------------------------------------|
| 01 | Temperature measurement    | Read temperature data from SHT31 sensor       |
| 02 | Humidity measurement       | Read humidity data from SHT31 sensor          |
| 03 | Light intensity measurement| Read light intensity data from BH1750 sensor  |

2. **Wireless Connectivity**
   
| ID | Functionality              | Note                                          |
|----|----------------------------|-----------------------------------------------|
| 01 | WiFi connection            | Connect ESP32 to local WiFi network           |
| 02 | MQTT connection            | Establish MQTT connection with Thingsboard server |

3. **Cloud Integration and Data Visualization**

| ID | Functionality              | Note                                          |
|----|----------------------------|-----------------------------------------------|
| 01 | Data publishing            | Send sensor data to Thingsboard via MQTT      |
| 02 | Dashboard display          | Visualize sensor data on Thingsboard dashboard|

4. **Alert System**
   
| ID | Functionality              | Note                                          |
|----|----------------------------|-----------------------------------------------|
| 01 | Rule chain configuration   | Set up alert conditions in Thingsboard        |
| 02 | Telegram bot integration   | Send alerts to user's Telegram bot            |

## Thingsboard Server Design

### Dashboard

![Dashboard](./dashboard.JPG)

The dashboard displays real-time environmental data, including:

- Humidity.
- Illuminance.
- Temperature.

It also shows an environmental parameter chart tracking temperature, humidity and light over time. The dashboard allows setting environmental parameters with input fields for maximum and minimum values of humidity, light, and temperature.

### Rule Chain

![Rule Chain](./rule_chain.JPG)

The rule chain processes incoming data and triggers actions based on predefined conditions:

1. Input node receives data
2. Key attributes are extracted
3. Three parallel "Under threshold" scripts evaluate the data
4. Based on the evaluation, alarms are either created or cleared for temperature, humidity and light
5. If alarms are created, a new Telegram message is composed
6. The message is sent via a REST API call to the user's Telegram bot

This rule chain enables automated alerting when environmental conditions exceed set thresholds.

## Setup Instructions

Before using this project, please follow these steps:

1. Open the `connect_wifi.h` file.
2. Add your WiFi network information (SSID and password) to the appropriate fields.
3. Save the file and proceed with compiling and uploading the firmware to your ESP32.

## Developed by Nguyen Thanh Minh