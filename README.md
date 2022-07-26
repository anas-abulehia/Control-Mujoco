# Control-Mujoco
Model Based Controller Using Mujoco 
### System Overview
The system contains 2 linkages connected with each other by rational joints. The first joint degree of freedom is around y axis. The second joint's degree of freedom is around the x axis. The controller is applying forces at the top mass in x and y direction (no z force is applied, no gravity cancellation too)  in order to reject any disturbance and keep the two masses vertically aligned.\
![image](https://user-images.githubusercontent.com/49106882/180995057-5db8b315-7ab1-4a3f-a435-9d2b858e7c53.png) \
The system uncontrolled response is shown [[here]](https://www.youtube.com/watch?v=wq-b49y99ZA].)
### Modelling 
The mechanical system is implemented via mujoco XML [reference](https://github.com/anas-abulehia/Control-Mujoco/blob/main/doublependulum.xml) in which we describe all properties of the system simulation, appearance, mass properties, joints parameters and settings. \
The details of modelling and equation of motion are found in the [[file]](https://github.com/anas-abulehia/Control-Mujoco/blob/main/Control_System_of_a_Grasp_of_A_crane.ipynb) or you can get a colab copy [[here]](https://colab.research.google.com/drive/1Ic6R3qBCWqHcgec2vXxMJ1mpeMv4kX9E?usp=sharing)\
### Controller
1) we got Euler Lagrange Equations from Modelling thanks to Sympy
2) we perform linearization at a point very closed to our desired final point, in our case theta1 and theta2 = 0.
3) Taking the inverse of The mass matrix and premultiply dynamics equations to get x_dot = Ax+Bu as the colab files shows
4) Checking the controllability matrix. It is fully ranked matrix.
5) Checking the observability matrix. It is full observable by measuring angular positions of theta_1 and theta_2
6) Define Q and R matrices
7) We can reach all states and model variables for a data structure. mjModel as specific in https://github.com/anas-abulehia/Control-Mujoco/blob/9a9df879b5cf2647c45fcc34346e0f10e3e428c4/main.c#L215-L223
8) Now we implement the controller, to make life harder we impose saturations, so we say 20 [N] the saturation affect the response time since the system is stable  https://github.com/anas-abulehia/Control-Mujoco/blob/f3687d39838539ae5a42511e2fad233ad9085eed/main.c#L241
9) Define our trajectory of both angles https://github.com/anas-abulehia/Control-Mujoco/blob/9a9df879b5cf2647c45fcc34346e0f10e3e428c4/main.c#L234-L235 
11) Find the control Law. LQR controller. Our Control Law is  ![image](https://user-images.githubusercontent.com/49106882/181115333-472f4e6f-72a7-413c-b898-4fec892acf1f.png)
12) Write the control law and apply the control inputs to the system via mjModel data structure https://github.com/anas-abulehia/Control-Mujoco/blob/9a9df879b5cf2647c45fcc34346e0f10e3e428c4/main.c#L225-L226 
https://github.com/anas-abulehia/Control-Mujoco/blob/9a9df879b5cf2647c45fcc34346e0f10e3e428c4/main.c#L258-L259

The response of the controlled system is [here](https://www.youtube.com/watch?v=ygOSoOh8YXE)
### Conculsion 
I have decided to work on Mujoco since it has been released and became open source project. The software shows an excellent performance in comparison to other  simulators [[1]](https://leggedrobotics.github.io/SimBenchmark/). The documentation was pretty clear and from convenience point of view, it is better than Matlab Simscape. In control design and mechanical dynamics simulation, it shows perfect results and I recommend it to be used in early development of projects, and it offers to add complexities such as sensor noises. It is perfect for teaching control and sure I will work on non-classical control problems  RL as a learning platform. The DeepMind team was very responsive and clear. All in all, the experience was very positive. 
At the end I would like to thank Dr Pranav Bhounsule [[2]](https://pab47.github.io/mujoco.html) for the nice tutorial which I found my way to Mujoco through it, indeed the C code and Makefile are downloaded from the course and modified for my project. 



