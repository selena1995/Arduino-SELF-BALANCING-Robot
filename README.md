# Arduino-SELF-BALANCING-Robot
A self-balancing robot is a two-wheeled robot that balances itself so that it prevents itself from falling.
Today in this tutorial, I have made a SELF-BALANCING robot with ARDUINO. I hope this article will help the beginners to make it properly

# BASICS ON SELF-BALANCING ROBOT

# WHAT IS A SELF-BALANCING ROBOT?

A self-balancing robot is a two-wheeled robot that balances itself so that it prevents itself from falling.


# HOW DOES A SELF-BALANCING ROBOT WORKS?

Self-balancing robots use a “closed-loop feedback control” system; this means that real-time data from motion sensors are used to control the motors and quickly compensate for any tilting motion in order to keep the robot upright. Similar self-balancing feedback control systems can be seen in many other applications.


# WHERE ARE SELF-BALANCING ROBOTS USED?

Among wheeled robots, two self-balancing robots, the Segway and Ninebot, have become popular and are used for commuting or as patrol transporters. In addition, self-balancing wheeled robots such as Anybots QB are currently used as a service robot platform.

# WHICH COMPONENTS WILL BE NEEDED AND WHY?
Here in this part, we will discuss the components, why we did choose them, and side by side we will make the robot too.

for a better explanation, I have divided this into three different parts-

Mechanical Part:- here we will learn to make the robot body in a very simple and easy way.
Electronics Part:- here we will learn about the electronics components, circuit designing, and assembling all the components. end of this part, our robot will be ready to go for the next and last level which is Arduino coding and calibration.
Arduino Coding & Calibration:- Here in this last part, we will discuss the Arduino code and how to calibrate the robot for self-balancing.
After completing these three steps, Our robot will be ready to show.


# MECHANICAL PART
We will start making our mechanical part first. For this, l have used some PVC sheets and 4 no of Graphite Pencil to make the robot Body. Please make the robot multi-storied because we are going to use mpu6050 which works best if it is placed at some height upper than the base. We will discuss more on mpu6050 in the later part of the electronics part of this article. For now, we will go for the mechanical part only.
After that, we will need two no of gear motors, two motor brackets, and two wheels.

![Screenshot (419)](https://user-images.githubusercontent.com/79990158/177102777-bbb919de-6894-4baf-9c2e-f0e78e408275.png)
![Screenshot (418)](https://user-images.githubusercontent.com/79990158/177102806-e1d22078-9932-4e08-ab00-d34d9b1b66d5.png)
![Screenshot (417)](https://user-images.githubusercontent.com/79990158/177102844-04f1e654-1218-4c55-b7ad-a0678f8a3972.png)
![Screenshot (416)](https://user-images.githubusercontent.com/79990158/177102874-a4904db4-e208-4af1-93db-aa20b2c7e9cb.png)
![Screenshot (415)](https://user-images.githubusercontent.com/79990158/177102900-63988e44-4490-4a70-b7af-031ef0c899ef.png)
![Screenshot (412)](https://user-images.githubusercontent.com/79990158/177102401-5878d365-d0e0-4adc-b15f-a2b7c8cbd727.png)
![Screenshot (413)](https://user-images.githubusercontent.com/79990158/177102488-a1b0150c-a1c8-48d9-8f4b-0aac9c02dfb2.png)![Screenshot (414)](https://user-images.githubusercontent.com/79990158/177102508-81d8b4c2-b07f-4f1e-a01c-df39bae834fb.png)
![Screenshot (423)](https://user-images.githubusercontent.com/79990158/177102617-e7010b40-5685-4b0d-862e-044528649573.png)
![Screenshot (422)](https://user-images.githubusercontent.com/79990158/177102671-7192a890-3b9a-47a1-9782-6fe9342b44c5.png)
![Screenshot (421)](https://user-images.githubusercontent.com/79990158/177102713-be49dd39-89f5-47aa-8fa3-b423778c90d5.png)
![Screenshot (420)](https://user-images.githubusercontent.com/79990158/177102750-acfb4f5b-440d-44a5-8510-3f93fa2cbbda.png)
I have used hot glue to attach all the parts together. It will take not more than 20-25min to give it a perfect look like this. Our mechanical part is ready and now we will go for the electronics part.

# ELECTRONICS PART
In this part, we will discuss the electronic components that we have used to make this robot. We will also learn why have we chosen all these components. Here we will also connect all the components together according to the circuit diagram that l have already attached with this article. Please download it before you are going to connect all the components together.

# ARDUINO NANO
Arduino NANO is the brain of the robot. Here l choose it because It's a perfect micro controller to learn hobby electronics and programming on, and its size makes it excellent for building into projects which require a small form factor.
![Screenshot (424)](https://user-images.githubusercontent.com/79990158/177104455-2ed18a86-782f-4de2-8f83-8eccd7ee5aa1.png)

# L298n Motor Driver
The L298N Motor Driver is a controller that uses an H-Bridge to easily control the direction and speed of up to 2 DC motors. The L298N is a dual H-Bridge motor driver which allows speed and direction control of two DC motors at the same time. The module can drive DC motors that have voltages between 5 and 35V, with a peak current up to 2A.
![Screenshot (425)](https://user-images.githubusercontent.com/79990158/177104642-9f81c4ff-835f-44ee-bd12-807132a3f13c.png)

# MPU6050
MPU6050 is a Micro Electro-mechanical system (MEMS), it consists of a three-axis accelerometer and a three-axis gyroscope. which means that it gives six values as output:

three values from the accelerometer
three from the gyroscope
It helps us to measure velocity, orientation, acceleration, displacement, and other motion-like features. It is very accurate, as it contains a 16-bits analog to digital conversion hardware for each channel. Therefore it captures the x, y, and z channels at the same time. The sensor uses the I2C-bus to interface with the Arduino. The MPU6050 can measure angular rotation using its on-chip gyroscope with four programmable full-scale ranges of ±250°/s, ±500°/s, ±1000°/s, and ±2000°/s.
![Screenshot (426)](https://user-images.githubusercontent.com/79990158/177104759-1fd7dfc1-af26-415a-98c0-d30fa543ac61.png)
These were the basic details of the components that I have used in this robot. There have many more components like Rocker switches, Jumper Wires, etc many more things which have no need for explanation l belief. Now l am going for the next step

# ASSEMBLE COMPONENTS TOGETHER
As I have already mentioned before I have already attached a circuit diagram with this article so please download that before you are going to connect all of them together. In my case, I have made a customized PCB board from JLCPCB to skip a few steps of wiring and make an easy connection between all the components.$2 for 5pcs PCBs, PCBA from $0, Register to Get Free Coupons here: https://jlcpcb.com/IYB. I really liked the PCB Boards as they are very high in quality.

![Screenshot (430)](https://user-images.githubusercontent.com/79990158/177105080-4c61d9e5-47ea-4595-a307-b72f0977727e.png)
![Screenshot (429)](https://user-images.githubusercontent.com/79990158/177105097-94ba7cd5-bbe7-4b80-8a3d-91d6b09151c1.png)
![Screenshot (433)](https://user-images.githubusercontent.com/79990158/177105110-2267228b-f55f-40e4-9ca1-870f1d8ec602.png)
![Screenshot (432)](https://user-images.githubusercontent.com/79990158/177105145-9355b4c9-2dfc-4825-ab93-5f0e8609d12b.png)
![Screenshot (431)](https://user-images.githubusercontent.com/79990158/177105158-b73796c8-48bb-4b26-b1fb-328485ed1583.png)

I have placed Arduino nano and MPU6050 on the same circuit board and also I have made a separate point for HC-05 Bluetooth Module so that I could easily upgrade my Self-Balancing Robot into a Bluetooth Controlled Self-Balanced Robot in my next project with the same PCB Board. I have just excluded the l298n motor driver module from my Customized PCB board. That portion will need to be wiring only.

![Screenshot (441)](https://user-images.githubusercontent.com/79990158/177113906-66ace8ab-ee86-496a-9253-82817cb76797.png)
![Screenshot (440)](https://user-images.githubusercontent.com/79990158/177113953-a897872a-76ac-4f2c-8cae-b82501d59cf0.png)
![Screenshot (438)](https://user-images.githubusercontent.com/79990158/177113984-db53b383-364f-429e-aa4c-0411d4849ae8.png)
![Screenshot (436)](https://user-images.githubusercontent.com/79990158/177114031-73ae67c9-f2a2-4286-a261-22c5218484b2.png)
![Screenshot (435)](https://user-images.githubusercontent.com/79990158/177114092-5a1a6603-a5b8-4f22-9928-9479d073b259.png)
![Screenshot (434)](https://user-images.githubusercontent.com/79990158/177114127-9e9393b9-d799-48c7-b80d-f13adb06c7ac.png)

here you can see that I have already done wiring the components together. The next step is very simple which is to collage the mechanical part and electronic part together and also have to connect a battery holder switch and 18650 li-ion battery with the robot body and the circuit. let's do that...
![Screenshot (450)](https://user-images.githubusercontent.com/79990158/177115228-d4378ed4-0a9a-424d-ae45-49c5dfeeea2d.png)
![Screenshot (449)](https://user-images.githubusercontent.com/79990158/177115262-c67911ee-1665-4ceb-a1a1-6907971c30bb.png)
![Screenshot (448)](https://user-images.githubusercontent.com/79990158/177115286-8d27f71f-0ec0-453f-883f-4144b57fe91e.png)
![Screenshot (447)](https://user-images.githubusercontent.com/79990158/177115318-da982838-b257-4454-875b-9f712bc17e4a.png)
![Screenshot (446)](https://user-images.githubusercontent.com/79990158/177115344-3c20f5ec-de7c-4995-b2a0-9b8a97b39b05.png)
![Screenshot (445)](https://user-images.githubusercontent.com/79990158/177115384-2eb4ba0d-f816-466d-b369-0e5171ce2906.png)
![Screenshot (443)](https://user-images.githubusercontent.com/79990158/177115400-bf42a382-08d8-4f7d-98a3-ebdc7391308b.png)

by installing all the components perfectly in the robot body, we have done making our robot. now only the last and final step has left to do which is Uploading Arduino code to the Arduino Nano.

# ARDUINO CODING AND CALIBRATION

There have to follow a few steps to calibrate & uploading the code...
for better understanding, l have divided this Arduino coding and calibration part into some different parts.
![FNP4PQ8J20PCI6N](https://user-images.githubusercontent.com/79990158/177115914-51950ca0-2fd7-49f3-9dc1-698f484bce81.png)
The MPU6050 has a 3-axis accelerometer and a 3-axis gyroscope. The accelerometer measures acceleration along the three axes and the gyroscope measures angular rate about the three axes. To measure the angle of inclination of the robot we need acceleration values along the y and z-axes. The atan2(y, z) function gives the angle in radians between the positive z-axis of a plane and the point given by the coordinates (z, y) on that plane, with the positive sign for counter-clockwise angles (right half-plane, y > 0), and negative sign for clockwise angles (left half-plane, y < 0). We use this library written by Jeff Rowberg to read the data from MPU6050. Upload the code given below and see how the angle of inclination varies.


#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
MPU6050 mpu;
int16_t accY, accZ;
float accAngle;
void setup() {  
  mpu.initialize();
  Serial.begin(9600);
}
void loop() {  
  accZ = mpu.getAccelerationZ();
  accY = mpu.getAccelerationY();
   
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  
  if(isnan(accAngle));
  else
    Serial.println(accAngle);
}
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
MPU6050 mpu;
int16_t accY, accZ;
float accAngle;
void setup() {  
  mpu.initialize();
  Serial.begin(9600);
}
void loop() {  
  accZ = mpu.getAccelerationZ();
  accY = mpu.getAccelerationY();
   
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  
  if(isnan(accAngle));
  else
    Serial.println(accAngle);
}


