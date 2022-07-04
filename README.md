# Arduino-SELF-BALANCING-Robot
A self-balancing robot is a two-wheeled robot that balances itself so that it prevents itself from falling.
Today in this tutorial, I have made a SELF-BALANCING robot with ARDUINO. I hope this article will help the beginners to make it properly

# Story

# BASICS ON SELF-BALANCING ROBOT

# WHAT IS A SELF-BALANCING ROBOT?

A self-balancing robot is a two-wheeled robot that balances itself so that it prevents itself from falling.


HOW DOES A SELF-BALANCING ROBOT WORKS?
Self-balancing robots use a “closed-loop feedback control” system; this means that real-time data from motion sensors are used to control the motors and quickly compensate for any tilting motion in order to keep the robot upright. Similar self-balancing feedback control systems can be seen in many other applications.


WHERE ARE SELF-BALANCING ROBOTS USED?
Among wheeled robots, two self-balancing robots, the Segway and Ninebot, have become popular and are used for commuting or as patrol transporters. In addition, self-balancing wheeled robots such as Anybots QB are currently used as a service robot platform.

WHICH COMPONENTS WILL BE NEEDED AND WHY?
Here in this part, we will discuss the components, why we did choose them, and side by side we will make the robot too.

for a better explanation, I have divided this into three different parts-

Mechanical Part:- here we will learn to make the robot body in a very simple and easy way.
Electronics Part:- here we will learn about the electronics components, circuit designing, and assembling all the components. end of this part, our robot will be ready to go for the next and last level which is Arduino coding and calibration.
Arduino Coding & Calibration:- Here in this last part, we will discuss the Arduino code and how to calibrate the robot for self-balancing.
After completing these three steps, Our robot will be ready to show.


MECHANICAL PART
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
