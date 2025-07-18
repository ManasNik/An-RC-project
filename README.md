# **An RC car project**
This is a work in progress personal project that I started with a goal of building an RC car from scratch with a gyroscopic steering controller imitating real life steering action and to gain experience with STM32 microcontrollers. My initial goal for this project is to complete the basic functioning of the car and steering control. I have planned and I am planning to add more features in the future such as controls to adjust certain parameters to affect the handling of the car, telemetry, lap time counter and even a camera to get a First Person View(FPV)!

## **Components used(so far)**
TB6612FNG x2   
STM32F103CBT6 x2  
Potentiometers for intial testing x2  
NRF24L01 x2

## **The initial breadboard circuit**
![Initial Prototype](https://github.com/user-attachments/assets/cb5571c8-4603-4a21-8e6c-857bef29882a)
(It's a mess of wires!)

I started my project by testing individual motors with the driver and eventually wired them all together. I initially tested using the STM32F446RE Nucleo board since it was readily available with me then. Obviously it's an overkill for this project and so I decided to use the STM32F103CBT6, which is still a powerful MCU for this project but leaves me headroom for adding interesting features later on in the future.
I used two potentiometers - one for speed control(throttle) and the other for steering control. This is the point I learned about DMA(Direct Memory Access) as it was needed to process the two ADC channels efficiently. Additionally I used the built-in button on the Nucleo board to enable/disable reverse under a specified speed. My idea is to implement the reverse button on the steering wheel controller later on in the future to engage/disengage reverse like in an F1 car! It has to be noted that the controller is not designed or involved in this initial setup - This is a hardwired test. 

I wanted to design a simple platform to hold my motors (the chassis) and small wheels to fit my desired form factor. I decided to design and 3D print my own design.

**Chassis design**
<img width="1919" height="910" alt="Chassis design" src="https://github.com/user-attachments/assets/40063d83-fbe2-4ff7-996f-b232247fefe2" />

**Wheel design**
<img width="1917" height="911" alt="Wheel design" src="https://github.com/user-attachments/assets/79d22599-8752-44b7-a575-50831f26888b" />

**The outcome of the design**
![Finished model parts](https://github.com/user-attachments/assets/4ba3a039-5642-4fbe-932a-60c43d2571dd)
(Placed a mouse there for comparison!)

# **Implementing remote control**
![Circuit with wireless control](https://github.com/user-attachments/assets/712097a4-f058-4d6b-83cc-bf60cb664994)  
(The car circuit is on the left and the controller on the right)

In this stage of the project I decided to add the remote control using two NRF24L01 modules - one on the car and one in the controller. To develop the driver for communicating with this module, I followed a YouTube tutorial and the NRF24l01 datasheet. This is the first time I wrote a driver/library by manipulating low level registers -  a great learning experience indeed! Once the driver development was done, it was time to test it. I shifted the two potentiometers and the reverse button to one of the two STM32F103CBT6 boards I will be using for the rest of this project and connected the NRF24L01 module via SPI. Similarly, I shifted the motor drivers to the other STM32F103CBT6 board and connected the other NRF24L01 module via SPI. I was now able to control the car through wireless communication!

# **What's next?**
(This section will be updated from time to time till the completion of the project)
So far the car wasn't tested on the ground as it is bounded to the breadboard by those jumper wires. I'm planning to design a prototype PCB with female headers to fit in these modules directly into them to keep the circuit clean and easy to manage while testing the car on the ground. Once the PCB design is over I can shift my focus to the key feature of my project - gyroscopic control which will be implemented using a MPU6050 IMU.



This project for me isn't just about building an RC car - it's a fun learning experience that will help me dive deep into complete embedded system design from the ground up.
