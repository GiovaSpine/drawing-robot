# Drawing Robot

This project is about a 3d printed **drawing robot** I made in my first year of computer engineering in 2023, to understand in more depth how a CNC machine works.  
It was 3D modeled in *FreeCAD* and programmed for an *Arduino Uno* entirely from scratch: from the classes that control the stepper motors, to the G-code functions that draw.

It was created before I started using proper version control, and I recently decided to publish it to share my work.


---


## The Robot

**The robot** is a SCARA manipulator with three degrees of freedom, that uses nema 17 stepper motors with GT2 belts and pulleys to reach a ratio of 20:1 for the first joint and 15:1 for the second joint.  
For the last joint we have a simple rack and pinion mechanism to convert the rotation of the stepper motor into a linear motion that moves the end effector (pen) to draw or not in the drawing plane.  

The code is divided in four *Arduino* files:
- [Scara_Manipulator_V1.ino](code/arduino/Scara_Manipulator_V1.ino): it's the main file where the classes for the stepper motors and the manipulator are declared, and the manipulator object is created.
- [CNCstepperClass.ino](code/arduino/CNCstepperClass.ino): where the class for the stepper motors is defined.
- [ManipulatorClass.ino](code/arduino/ManipulatorClass.ino): where the manipulator class is defined, with its functions that use the inverse kinematics to control the robot.
- [FunzioniGCODE.ino](code/arduino/FunzioniGCODE.ino): where the G-code functions of the manipulator class are defined.  

  
The following files allow to visualize the robot in 3D:
- *Blender* file (created with Blender 4.5): [Scara Manipulator.blend](3D-models/Scara%20Manipulator.blend)
- *.stl* file: [Scara Manipulator.stl](3D-models/pezzi/components/Scara%20Manipulator.stl)

![Robot Frontale](images/Scara.JPG)


---


## Robot in Action

The robot draws a butterfly.  

![Robot in action 1](images/Video3.gif)

The robot draws a smiling face.  

![Robot in action 3](images/Video2.gif)

The robot draws the letter "A".  

![Robot in action 2](images/Video1.gif)



---


## Components
The robot was made in *FreeCAD* and designed to be able to be 3D printed in PLA with a cheap 3D printer.  
In addition we need the following components:

![Componenti Robot](images/components.jpg)


---


## Robot Diagram
The robot has a very simple diagram, and in particular, since the objective is to draw, a complicated control for the $z$ axis is not needed, and so we can have two values for $z$: the value where the end effector (pen) is touching the drawing plane, and the value where it is not.  
As a result only the measurements for the horizontal links $d_2$ and $d_4$ are shown.

![Robot Diagram 1](images/DiagrammaManipolatore.png)
![Robot Diagram 2](images/Misure.png)


---


## Inverse Kinematics
The **inverse kinematics** is very simple, not only because we have a three degrees of freedom manipulator, but also because with the simplification of the $z$ axis, discussed earlier, only $d_2$ and $d_4$ are used in the calculations for $\theta_1$ and $\theta_2$ (the angles of the first two joints respectively).  
For $\theta_3$ we need only to know the two values that allow the end effector (pen) to touch or not the drawing plane.

In the following image, two kinematic diagrams are shown. They represent a top view of the robot: one corresponds to a configuration where the desired $y_0$ is greater than 0, and the other where it is less than 0.
The equations derived using basic trigonometry are also included.
![Inverse Kinematics](images/CinematicaInversa.svg)
