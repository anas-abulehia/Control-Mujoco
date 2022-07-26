# Control-Mujoco
Model Based Controller Using Mujoco 
### System Overview
The system contains 2 linkages connected with each other by rational joints. The first joint degree of freedom is around y axis. The second joint's degree of freedom is around the x axis. The controller is applying forces in x and y direction in order to reject any noise and keep the two masses vertically aligned.\
![image](https://user-images.githubusercontent.com/49106882/180995057-5db8b315-7ab1-4a3f-a435-9d2b858e7c53.png) \
The system unconrtolled response is shown [[here]](https://www.youtube.com/watch?v=wq-b49y99ZA].)
### Modelling 
The mechanical system is implemented via mujoco XML reference in which we describe all properties of the system simulation, appearance, mass properties, joints parameters and settings. \
The details of modelling and equation of motion are found in the [[file]](https://github.com/anas-abulehia/Control-Mujoco/blob/main/Control_System_of_a_Grasp_of_A_crane.ipynb) or you can get a colab copy [[here]](https://colab.research.google.com/drive/1Ic6R3qBCWqHcgec2vXxMJ1mpeMv4kX9E?usp=sharing)
