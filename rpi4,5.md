# Tutorial Workbook Series: Raspberry Pi 4/5 for Students

---

## Introduction

Welcome to the world of single-board computing with the Raspberry Pi! This workbook is your guide to mastering the Raspberry Pi 4 and 5, two powerful and versatile computers packed into a credit-card-sized board. Whether you are a student, hobbyist, or aspiring technologist, this guide will walk you through the essentials of setting up your Raspberry Pi, understanding its capabilities, and building exciting, hands-on projects.

**What is a Raspberry Pi?**
The Raspberry Pi is a low-cost, high-performance computer designed to teach programming, electronics, and problem-solving. The Raspberry Pi 4 and 5 are capable of tasks you'd expect from a desktop computer, from browsing the internet and playing high-definition video to being the brain of a robot or a smart home hub.

**What You Will Learn:**
By the end of this workbook, you will be able to:
*   Set up and configure a Raspberry Pi 4 or 5.
*   Navigate the Raspberry Pi OS and use basic Linux commands.
*   Understand the function of GPIO pins to interface with electronic components.
*   Write Python scripts to control hardware.
*   Build and deploy 10 different projects, ranging from simple circuits to complex IoT systems.

---

## 1. Raspberry Pi 4/5 Overview

This section details the hardware specifications of the Raspberry Pi 4 and 5 and the components you will need to get started.

### Hardware Specifications

While both boards share the same 40-pin GPIO layout, the Raspberry Pi 5 offers a significant performance boost. [3, 6, 15]

| Feature | Raspberry Pi 4 Model B | Raspberry Pi 5 |
| :--- | :--- | :--- |
| **Processor** | Quad-core Cortex-A72 @ 1.5-1.8GHz. [3] | Quad-core Cortex-A76 @ 2.4GHz. [3] |
| **RAM** | 2GB, 4GB, or 8GB LPDDR4. [3] | 4GB or 8GB LPDDR4X (more power efficient). [3] |
| **Connectivity** | Wi-Fi (802.11ac), Bluetooth 5.0, Gigabit Ethernet. [6] | Wi-Fi (802.11ac), Bluetooth 5.0, Gigabit Ethernet. [6] |
| **USB Ports** | 2x USB 3.0, 2x USB 2.0. [3] | 2x USB 3.0 (supports simultaneous 5Gbps), 2x USB 2.0. [3] |
| **Video Output**| 2x micro-HDMI ports (up to 4Kp60). [15] | 2x micro-HDMI ports (up to dual 4Kp60). [15] |
| **Storage** | MicroSD card slot. [3] | High-speed microSD card slot (SDR104 support). [3] |
| **Special Features**| - | **PCIe 2.0 x1 interface**, on-board power button, Real-Time Clock (RTC). [4, 6, 14, 16] |
| **Power** | 5V/3A via USB-C. [3] | **5V/5A** via USB-C (Power Delivery). [2, 5] |

### Required Components

*   **Raspberry Pi 4 or 5:** The core of your projects.
*   **Power Supply:**
    *   For Raspberry Pi 4: A high-quality **5V/3A** USB-C power supply. [3]
    *   For Raspberry Pi 5: The official **5V/5A** USB-C power supply is highly recommended to ensure full functionality, especially for the USB ports. [5, 8] Using a lower amperage supply will limit the power to peripherals. [8, 21]
*   **MicroSD Card:** A high-speed (Class 10 or faster), 16GB or larger card.
*   **Peripherals:** USB Keyboard, USB Mouse, and a monitor with an HDMI port.
*   **Cables:** A micro-HDMI to HDMI cable (or two for a dual-display setup).
*   **(Optional but Recommended for RPi 5):** Active Cooler or a case with a fan to manage heat during intensive tasks.

---

## 2. Getting Started: Setup and Configuration

This section will guide you through setting up your Raspberry Pi for the first time.

### Hardware Setup

1.  **Install Heatsink/Cooler (Recommended):** Especially for the Pi 5, attach the active cooler or heatsinks to the main chips to prevent overheating.
2.  **Mount in Case:** Place your Raspberry Pi into its official case or a third-party alternative.
3.  **Connect Peripherals:**
    *   Insert the microSD card (with the OS installed, see below) into the slot. [9, 13]
    *   Connect your keyboard and mouse to the USB ports. [9, 13]
    *   Connect your monitor to one of the micro-HDMI ports. [9, 13]
4.  **Power Up:** Connect the USB-C power supply to the Pi. For the Raspberry Pi 5, press the onboard power button to turn it on. [7]

### Software Setup: Installing Raspberry Pi OS

The easiest way to install the operating system is with the **Raspberry Pi Imager**. [12, 32, 34, 35]

1.  **Download Imager:** On another computer, go to `raspberrypi.com/software` and download the Raspberry Pi Imager. [29]
2.  **Choose a Device and OS:**
    *   Open the Imager and click "Choose Device" to select your Raspberry Pi model (4 or 5). [32]
    *   Click "Choose OS" and select **Raspberry Pi OS (64-bit)** for the best performance. [9]
3.  **Select Storage:** Insert your microSD card into your computer and select it under "Choose Storage." [32]
4.  **Configure and Write:**
    *   Click "Next" and then "Edit Settings" to pre-configure a username/password, enable SSH (for remote access), and set up your Wi-Fi credentials. This makes the first boot much smoother. [35]
    *   Click "Yes" to begin writing the OS to the card. This will erase all data on the card.
5.  **First Boot:** Once writing is complete, eject the microSD card, insert it into your Pi, and power it on. The system will guide you through an initial setup if you didn't pre-configure it. [13]

### System Updates

Once your Pi has booted and is connected to the internet, open a **Terminal** window and run the following commands to ensure your system is up-to-date:

```bash
sudo apt update
sudo apt upgrade -y
```

---

## 3. Safety and Best Practices

*   **Power Down Properly:** Always shut down your Raspberry Pi using the menu or the `sudo shutdown now` command in the terminal before unplugging it.
*   **GPIO Voltage:** The GPIO pins operate at **3.3V**. Connecting a 5V source directly to a GPIO pin can permanently damage your Raspberry Pi. [33]
*   **Use Resistors:** Always use a current-limiting resistor (like 220Ω or 330Ω) when connecting LEDs to GPIO pins to avoid burning them out.
*   **Backup Your Work:** Regularly back up your microSD card to prevent data loss.

---

## 4. Projects Section

Here are 10 hands-on projects to get you started. Each project builds on the skills learned in the previous one.

### Project 1: LED Blinker (Hello, World of Hardware)

*   **Objective:** Learn to control a GPIO pin using Python.
*   **Materials:** 1x LED, 1x 330Ω Resistor, 2x Jumper Wires, 1x Breadboard.
*   **Instructions:**
    1.  Connect the longer leg (anode) of the LED to a row on the breadboard.
    2.  Connect the 330Ω resistor from the same row to another point on the breadboard.
    3.  Use a jumper wire to connect the other end of the resistor to **GPIO 18** (Physical Pin 12).
    4.  Use another jumper wire to connect the shorter leg (cathode) of the LED to a **Ground (GND)** pin (Physical Pin 6).
    5.  Write and run the following Python script using Thonny IDE.
*   **Code (`blink.py`):**
    ```python
    import RPi.GPIO as GPIO
    import time

    GPIO.setmode(GPIO.BCM) # Use Broadcom pin numbering
    LED_PIN = 18
    GPIO.setup(LED_PIN, GPIO.OUT)

    try:
        while True:
            GPIO.output(LED_PIN, GPIO.HIGH) # Turn LED ON
            time.sleep(1)
            GPIO.output(LED_PIN, GPIO.LOW)  # Turn LED OFF
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup() # Clean up GPIO on CTRL+C exit
    ```
*   **Learning Outcomes:** GPIO pin control, basic Python syntax, and simple circuit building.
*   **Challenge:** Modify the script to make the LED blink in an S.O.S. pattern.

---

### Project 2: Push Button-Controlled LED

*   **Objective:** Read a digital input from a push button.
*   **Materials:** Project 1 components + 1x Push Button.
*   **Instructions:**
    1.  Place the push button on the breadboard.
    2.  Connect one leg of the button to **GPIO 17** (Pin 11).
    3.  Connect the diagonally opposite leg to a **3.3V** pin (Pin 1).
*   **Code (`button_led.py`):**
    ```python
    import RPi.GPIO as GPIO

    GPIO.setmode(GPIO.BCM)
    LED_PIN = 18
    BUTTON_PIN = 17

    # Setup LED as output and Button as input with a pull-down resistor
    GPIO.setup(LED_PIN, GPIO.OUT)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    try:
        while True:
            if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
                GPIO.output(LED_PIN, GPIO.HIGH)
            else:
                GPIO.output(LED_PIN, GPIO.LOW)
    except KeyboardInterrupt:
        GPIO.cleanup()
    ```
*   **Learning Outcomes:** Reading digital inputs, using internal pull-up/pull-down resistors.
*   **Challenge:** Make the LED stay on after one press and turn off with a second press (a toggle switch).

---

### Project 3: Temperature and Humidity Monitor

*   **Objective:** Interface with a sensor to read environmental data.
*   **Materials:** 1x DHT11 or DHT22 Sensor, Jumper Wires.
*   **Instructions:**
    1.  Install the Adafruit DHT library: `pip install Adafruit_DHT`.
    2.  Connect the DHT sensor's VCC pin to 3.3V, GND to GND, and Data pin to **GPIO 4** (Pin 7).
*   **Code (`read_temp.py`):**
    ```python
    import Adafruit_DHT
    import time

    SENSOR = Adafruit_DHT.DHT11
    SENSOR_PIN = 4

    while True:
        humidity, temperature = Adafruit_DHT.read_retry(SENSOR, SENSOR_PIN)
        if humidity is not None and temperature is not None:
            print(f"Temperature={temperature:.1f}°C  Humidity={humidity:.1f}%")
        else:
            print("Failed to retrieve data from sensor")
        time.sleep(2)
    ```
*   **Learning Outcomes:** Using third-party Python libraries, interfacing with sensors.
*   **Challenge:** Log the data to a CSV file with timestamps.

---

### Project 4: Web-Controlled LED (Simple IoT)

*   **Objective:** Create a simple web server to control hardware remotely.
*   **Materials:** Project 1 components.
*   **Instructions:**
    1.  Install Flask: `pip install flask`.
    2.  Create a web page that turns the LED on or off.
*   **Code (`app.py`):**
    ```python
    from flask import Flask, render_template_string
    import RPi.GPIO as GPIO

    app = Flask(__name__)
    GPIO.setmode(GPIO.BCM)
    LED_PIN = 18
    GPIO.setup(LED_PIN, GPIO.OUT)

    @app.route('/')
    def index():
        return render_template_string("""
        <html><body>
        <h1>Web Controlled LED</h1>
        <a href="/led/on">Turn On</a><br>
        <a href="/led/off">Turn Off</a>
        </body></html>
        """)

    @app.route('/led/<state>')
    def led_control(state):
        if state == 'on':
            GPIO.output(LED_PIN, GPIO.HIGH)
        else:
            GPIO.output(LED_PIN, GPIO.LOW)
        return "LED is " + state

    if __name__ == '__main__':
        app.run(host='0.0.0.0', port=80)
    ```
*   **Learning Outcomes:** Basic web server development with Flask, IoT principles.
*   **Challenge:** Add a status indicator to the webpage showing the current state of the LED.

---

### Project 5: Motion-Activated Camera

*   **Objective:** Build a security camera that captures an image when motion is detected.
*   **Materials:** Raspberry Pi Camera Module or USB Webcam, 1x PIR Motion Sensor.
*   **Instructions:**
    1.  Enable the camera interface using `sudo raspi-config`.
    2.  Install the `picamera` library: `pip install picamera`.
    3.  Connect the PIR sensor's VCC to 5V, GND to GND, and OUT to **GPIO 23** (Pin 16).
*   **Code (`security_cam.py`):**
    ```python
    from picamera import PiCamera
    import RPi.GPIO as GPIO
    import time

    camera = PiCamera()
    GPIO.setmode(GPIO.BCM)
    PIR_PIN = 23
    GPIO.setup(PIR_PIN, GPIO.IN)

    print("Ready...")
    try:
        while True:
            if GPIO.input(PIR_PIN):
                print("Motion Detected!")
                filename = f"capture-{int(time.time())}.jpg"
                camera.capture(filename)
                print(f"Image captured: {filename}")
                time.sleep(5) # Wait 5 seconds before next detection
    except KeyboardInterrupt:
        GPIO.cleanup()
    ```
*   **Learning Outcomes:** Using the Pi Camera, event-driven programming.
*   **Challenge:** Send an email with the captured image as an attachment.

---

### Project 6: Retro Gaming Console with RetroPie

*   **Objective:** Turn your Raspberry Pi into a retro gaming machine.
*   **Materials:** USB Game Controller, MicroSD card (dedicated for this project).
*   **Instructions:**
    1.  Download the correct RetroPie image for your Pi model from `retropie.org.uk`.
    2.  Use the Raspberry Pi Imager to flash the RetroPie image to your microSD card.
    3.  Boot up the Pi, and RetroPie will guide you through configuring your controller.
    4.  Transfer game ROMs (use legally owned copies) to the appropriate folders via a USB stick or network share.
*   **Learning Outcomes:** Installing a different OS, file management, and software configuration.
*   **Challenge:** Customize the look and feel of your RetroPie system with different themes.

---

### Project 7: Smart Weather Station with ThingSpeak

*   **Objective:** Collect weather data and visualize it on an online dashboard.
*   **Materials:** BME280 Sensor (provides temperature, humidity, and pressure).
*   **Instructions:**
    1.  Sign up for a free account on `thingspeak.com` and create a new channel with 3 fields (temp, humidity, pressure). Note your "Write API Key".
    2.  Enable I2C in `sudo raspi-config`.
    3.  Connect the BME280 sensor to the I2C pins (SDA to GPIO 2, SCL to GPIO 3).
    4.  Install required libraries: `pip install adafruit-circuitpython-bme280 requests`.
*   **Code (`weather_station.py`):**
    ```python
    import time
    import board
    import adafruit_bme280.advanced as adafruit_bme280
    import requests

    API_KEY = 'YOUR_THINGSPEAK_API_KEY'
    i2c = board.I2C()
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

    while True:
        temp = bme280.temperature
        humidity = bme280.humidity
        pressure = bme280.pressure
        
        payload = {'api_key': API_KEY, 'field1': temp, 'field2': humidity, 'field3': pressure}
        try:
            requests.get('https://api.thingspeak.com/update', params=payload)
            print("Data sent to ThingSpeak")
        except:
            print("Connection failed")
        
        time.sleep(60) # Send data every minute
    ```
*   **Learning Outcomes:** Using the I2C protocol, sending data to a cloud service (IoT), API usage.
*   **Challenge:** Set up an alert on ThingSpeak to notify you if the temperature goes above a certain threshold.

---

### Project 8: Home Automation with a Relay

*   **Objective:** Control a low-power appliance (like a desk lamp) using a relay.
*   **Materials:** 1-Channel Relay Module, a lamp you can safely modify, jumper wires.
*   **WARNING:** Working with mains voltage is dangerous. This project should only be done with low-voltage devices (e.g., a 12V fan) unless supervised by someone experienced with electronics.
*   **Instructions:**
    1.  Connect the relay module's VCC to 5V, GND to GND, and IN to **GPIO 26** (Pin 37).
    2.  Carefully cut one of the two wires of the lamp's power cord and connect the two ends to the COM and NO (Normally Open) terminals of the relay.
*   **Code (`relay_control.py`):**
    ```python
    import RPi.GPIO as GPIO
    import time

    GPIO.setmode(GPIO.BCM)
    RELAY_PIN = 26
    GPIO.setup(RELAY_PIN, GPIO.OUT)

    # Relay is often active-low, so HIGH is OFF and LOW is ON
    try:
        print("Turning lamp ON")
        GPIO.output(RELAY_PIN, GPIO.LOW)
        time.sleep(5)
        print("Turning lamp OFF")
        GPIO.output(RELAY_PIN, GPIO.HIGH)
    finally:
        GPIO.cleanup()
    ```
*   **Learning Outcomes:** Understanding and using relays to control higher-power devices.
*   **Challenge:** Integrate this script with the Flask web server from Project 4 to control the lamp from a webpage.

---

### Project 9: Robotic Car

*   **Objective:** Build and program a simple, two-wheeled robotic car.
*   **Materials:** Robot car chassis kit (with 2 DC motors), L298N Motor Driver, Battery pack (e.g., 4xAA).
*   **Instructions:**
    1.  Assemble the robot chassis.
    2.  Connect the battery pack to the L298N's power terminals.
    3.  Connect the L298N's 5V output to the Pi's 5V pin and GND to a GND pin.
    4.  Connect the motor driver inputs (e.g., IN1, IN2, IN3, IN4) to four GPIO pins (e.g., 5, 6, 13, 19).
    5.  Connect the motors to the motor outputs on the driver board.
*   **Code (simplified forward motion):**
    ```python
    import RPi.GPIO as GPIO
    import time

    GPIO.setmode(GPIO.BCM)
    # Define motor driver pins
    IN1, IN2 = 5, 6  # Left motor
    IN3, IN4 = 13, 19 # Right motor
    GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT)

    def forward():
        GPIO.output([IN1, IN3], GPIO.HIGH)
        GPIO.output([IN2, IN4], GPIO.LOW)

    def stop():
        GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)

    try:
        print("Moving forward for 3 seconds")
        forward()
        time.sleep(3)
    finally:
        stop()
        GPIO.cleanup()
    ```
*   **Learning Outcomes:** Motor control, principles of robotics.
*   **Challenge:** Add functions for `backward()`, `left()`, and `right()` and control the robot with keyboard input.

---

### Project 10: AI Image Classifier (Raspberry Pi 5 Recommended)

*   **Objective:** Use TensorFlow Lite to classify objects seen by the camera in real-time. This project leverages the enhanced processing power of the Raspberry Pi 5. [10]
*   **Materials:** Raspberry Pi Camera Module.
*   **Instructions:**
    1.  This is an advanced project. Start by setting up the TensorFlow Lite example repository. Open a terminal and run:
        ```bash
        git clone https://github.com/tensorflow/examples.git
        cd examples/lite/examples/object_detection/raspberry_pi
        sh setup.sh
        ```
    2.  This script will download the necessary models and install dependencies. [30]
    3.  Run the classifier. [26, 27]
        ```bash
        python3 detect.py --model efficientdet_lite0.tflite
        ```
*   **Expected Outcome:** A window will appear showing the camera feed with boxes drawn around detected objects (e.g., 'person', 'cup', 'keyboard').
*   **Learning Outcomes:** Introduction to machine learning on edge devices, real-time image processing, using pre-trained AI models. [23]
*   **Challenge:** Modify the code to trigger an action when a specific object (like a banana) is detected—for example, by lighting up an LED.

---

## 5. Resources and Further Reading

*   **Official Documentation:** The best place to start for any questions.
    *   [Raspberry Pi Documentation](https://www.raspberrypi.com/documentation/)
    *   [Raspberry Pi Hardware](https://www.raspberrypi.com/products/)
*   **Communities:**
    *   Official Raspberry Pi Forums: `forums.raspberrypi.com`
    *   The Raspberry Pi Subreddit: `reddit.com/r/raspberry_pi`
    *   Search for `#RaspberryPi` or `#RPi5` on social media for project ideas.
*   **Helpful Websites:**
    *   [Raspberry Pi Stack Exchange](https://raspberrypi.stackexchange.com/)
    *   [MagPi Magazine](https://magpi.raspberrypi.com/) (Free PDF downloads)

---

## 6. Appendices

### Appendix A: GPIO Pinout Diagram

This is the standard 40-pin header layout for the Raspberry Pi 4 and 5. [25, 28, 31]

```
       3.3V  (1) (2)  5V
SDA (I2C)  (3) (4)  5V
SCL (I2C)  (5) (6)  GND
   GPIO 4  (7) (8)  TXD (UART)
        GND  (9) (10) RXD (UART)
  GPIO 17 (11) (12) GPIO 18 (PWM)
  GPIO 27 (13) (14) GND
  GPIO 22 (15) (16) GPIO 23
       3.3V (17) (18) GPIO 24
 MOSI(SPI) (19) (20) GND
 MISO(SPI) (21) (22) GPIO 25
 SCLK(SPI) (23) (24) CE0 (SPI)
        GND (25) (26) CE1 (SPI)
EEPROM SDA (27) (28) EEPROM SCL
   GPIO 5 (29) (30) GND
   GPIO 6 (31) (32) GPIO 12 (PWM)
  GPIO 13 (33) (34) GND
MISO(SPI) (35) (36) GPIO 16
  GPIO 19 (37) (38) MOSI(SPI)
        GND (39) (40) SCLK(SPI)
```

### Appendix B: Common Linux Commands

*   `ls`: List files and directories.
*   `cd [directory]`: Change directory.
*   `pwd`: Print working directory (shows your current location).
*   `sudo [command]`: Run a command with administrative privileges.
*   `nano [filename]`: Open a simple text editor.
*   `python3 [filename.py]`: Run a Python script.
*   `pinout`: Displays a GPIO pinout diagram in the terminal.

---
**End of Workbook**