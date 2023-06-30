# LICENSE

Shield: [![CC BY-NC 4.0][cc-by-nc-shield]][cc-by-nc]

This work is licensed under a
[Creative Commons Attribution-NonCommercial 4.0 International License][cc-by-nc].

[![CC BY-NC-SA 4.0][cc-by-nc-image]][cc-by-nc]

[cc-by-nc]: http://creativecommons.org/licenses/by-nc/4.0/
[cc-by-nc-image]: https://licensebuttons.net/l/by-nc/4.0/88x31.png
[cc-by-nc-shield]: https://img.shields.io/badge/License-CC%20BY--NC%204.0-lightgrey.svg

# ROS and Experimental Robotics

This repository gathers (most of) the pedagogical ressources used during the Sorbonne Université teaching unit called "ROS and experimental robotics", which is mainly dedicated to [ROS](https://www.ros.org/) (a software framework for Robotics) and its use in a simulated or experimental context. 
This unit is proposed in the ["Automatique, Robotique" Master]([https://www.google.com](https://sciences.sorbonne-universite.fr/formation-sciences/offre-de-formation/masters/master-automatique-robotique)), 1st year, and is common to the ["Intelligent System"](https://sciences.sorbonne-universite.fr/formation-sciences/offre-de-formation/masters/master-automatique-robotique/parcours-ingenierie-des), ["Advanced Systems and Robotics"](https://sciences.sorbonne-universite.fr/formation-sciences/offre-de-formation/masters/master-automatique-robotique/parcours-systemes) and ["Mechanical Systems for rehabilitation"](https://sciences.sorbonne-universite.fr/formation-sciences/offre-de-formation/masters/master-electronique-energie-electrique-automatique-1) paths.

This teaching unit mainly relies on a pre-installed virtual machine provided by the teachers at the very beggining of the semester (it is not available in this repository, but can me made available upon request). A short [how-to](Practicals/Installation/installation.pdf) is provided to explain how to use this VM, based on standard Ubuntu 20.04 and Ros Kinetic installations. Fur students working with macOS, an alternative installation method based on [RoboStack](https://robostack.github.io/) is proposed [in the repository](Practicals/macOS/Robostack_on_macOS.pdf)

The students have access to about 30 [turtlebot 3 burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/). Each robot emits a wifi hotspot with a SSID ***turtlebotX_wifi***, with $X$ the number of the robot sticked on the higher plate. The way to interact with the robot through ROS (wifi connection, network parameterization of the VM, etc.) is indicated in the [turtlebot guide](Practicals/Turtlebot_guide/turtlebot_guide.pdf) document.

## Pedagogical approach
The unit is mainly rooted on the ABCD (Activity Based Curriculum Design) method. Basically, this approach relies on:
- A student-centered learning: teaching activities are designed in accordance with students' needs to achieve learning objectives;
- Evaluations that are integrated into the learning process;
- Active Learning: lecture design includes participatory methods that encourage the learning ability of future professionals, shared among the following activities:
  - Acquisition
  - Investiation/research
  - Practice/training
  - Discussion
  - Production
  - Collaboration

In addition, we tried to be careful about well-formulated learning objectives:  
- concise and precise
- quantifiable (by the student and for assessment)
- focused on skills 
with aligned evaluations which:
- are part of the learning process;
- and should only assess the learning objectives e.g. "don’t test the application of a skill if it has not been taught".

Then, our main pedagogical objective is to "make students familiar with ROS and ready to develop with it".
Our second objectives are:
- develop an application based on a set of specifications 
- confront the differences between simulation and a real robot
- apply theoretical knowledge of robotics and exploit all competencies from past/actual teaching units: in Python, Signal/image processing, Automatic control.

## Practicals
This folder contains the source files of the LateX-generated PDF file used during the practicals sessions. *An enriched version, listing all the answers of each question is also available upon request.*
The unit is split in 3 different parts:
- **[Part 1](Practicals/Part_1), an introduction to ROS**: this is where all the students actually discover ROS and its root concept. Students must carefully follow online tutorials, helped with a guide and a forum available on the Sorbonne Université Moodle unit page;
- **[Part 2](Practicals/Part_2), URDF and simulations with ROS**: this is where the students start to work with [gazebo](https://gazebosim.org/), a physical simuator used to gather realistic sensorimotor data from simulations;
- **[Part 3](Practicals/Part_3), project**: the students now work on their own, on simulation *and* with a real [turtlebot 3 burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) system. They all have different challenges to reach, like navigating by following color lines, avoiding obstacles, navigating in a corridor, etc.

## Evaluation
The evaluation is made of:
- two MCQs (on Moodle); *the raw files listing all the questions (and answers!) are available upon request;*
- one practical exam: it is different each year; the students have to code in a limited time a ROS node with specific specifications; *the LateX subject and the code used for the two evaluations in the last two years are available in the [Evaluation/Exam](Evaluation/Exam) folder. The correction can be shared upon request;*
- the project, which is evaluated on the simulated and experimental parts, with a special care to the code written during these sessions.

---

To contact us: (add sorbonne-universite.fr to our email to reach us!)
- Sylvain Argentieri <sylvain.argentieri@>
- Fabien Vérité <fabien.verite@>
- Ludovic Saint Bauzel <ludovic.saint-bauzel@>
