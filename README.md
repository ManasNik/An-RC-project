# **An RC car project**
This is a work in progress personal project that I started with a goal of building an RC car from scratch with a gyroscopic steering controller imitating real life steering action and to gain experience with STM32 microcontrollers. My initial goal for this project is to complete the basic functioning of the car and steering control. I have planned and I am planning to add more features in the future such as controls to adjust certain parameters to affect the handling of the car, telemetry, lap time counter and even a camera to get a First Person View(FPV)!

## **Components using**
TB6612FNG x2   
STM32F103CBT6 x2  
Potentiometers for intial testing x2  
NRF24L01 x2

## **The initial breadboard circuit**
![Initial Prototype](https://github.com/user-attachments/assets/cb5571c8-4603-4a21-8e6c-857bef29882a)
(It's a mess of wires!)

I started my project by testing the individual motor with the driver and eventually wired them all together. I initially tested using the STM32F446RE Nucleo board since it was readily available with me then. Obviously it's an overkill for this project and so I decided to use the STM32F103CBT6, which is still a powerful MCU for this project but gives headroom for adding interesting features later on in the future.
I used two potentiometers - one for speed control(throttle) and the other for steering control. This is the point I learned about DMA(Direct Memory Access) as it was needed to process the two ADC channels efficiently. It has to be noted that the controller is not designed or involved in this initial setup. This is a hardwired test. 

I wanted to design a simple platform to hold my motors (the chassis) and small wheels to fit my desired form factor. I decided to design and 3D print my own design.

**Chassis design**
<img width="1919" height="910" alt="Chassis design" src="https://github.com/user-attachments/assets/40063d83-fbe2-4ff7-996f-b232247fefe2" />

**Wheel design**
<img width="1917" height="911" alt="Wheel design" src="https://github.com/user-attachments/assets/79d22599-8752-44b7-a575-50831f26888b" />
