# ROS and Experimental Robotics

This repository gathers (most of) the pedagogical ressources used during the Sorbonne Université teaching unit called "ROS and experimental robotics", which is mainly dedicated to [ROS](https://www.ros.org/) (a software framework for Robotics) and its use in a simulated or experimental context. 
This unit is proposed in the ["Automatique, Robotique" Master]([https://www.google.com](https://sciences.sorbonne-universite.fr/formation-sciences/offre-de-formation/masters/master-automatique-robotique)), 1st year, and is common to the ["Intelligent System"](https://sciences.sorbonne-universite.fr/formation-sciences/offre-de-formation/masters/master-automatique-robotique/parcours-ingenierie-des), ["Advanced Systems and Robotics"](https://sciences.sorbonne-universite.fr/formation-sciences/offre-de-formation/masters/master-automatique-robotique/parcours-systemes) and ["Mechanical Systems for rehabilitation"](https://sciences.sorbonne-universite.fr/formation-sciences/offre-de-formation/masters/master-electronique-energie-electrique-automatique-1) paths.


## Practicals
This folder contains the source files of the LateX-generated PDF file used during the practicals sessions. *An enriched version, listing all the answers of each question is also available upon request.*
The unit is split in 3 different parts:
- **[Part 1](Practicals/Part_1), an introduction to ROS**: this is where all the students actually discover ROS and its root concept. Students must carefully follow online tutorials, helped with a guide and a forum available on the Sorbonne Université Moodle unit page;
- **Part 2, URDF and simulations with ROS**: this is where the students start to work with [gazebo](https://gazebosim.org/), a physical simuator used to gather realistic sensorimotor data from simulations;
- **Part 3, project**: the students now work on their own, on simulation *and* with a real [turtlebot 3 burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) system. They all have different challenges to reach, like navigating by following color lines, avoiding obstacles, navigating in a corridor, etc.

## Evaluation
The evaluation is made of:
- two MCQs (on Moodle); *the raw files listing all the questions (and answers!) are available upon request;*
- one practical exam: it is different each year; the students have to code in a limited time a ROS node with specific specifications; *the LateX subject and the code used for the two evaluations in the last two years are available in the [Evaluation/Exam](Evaluation/Exam) folder.*
- the project, which is evaluated on the simulated and experimental parts, with a special care to the code written during these sessions.

---

This teaching unit mainly relies on a pre-installed virtual machine provided by the teachers at the very beggining of the semester. A short [how-to](Practicals/Installation/installation.pdf) is provided to explain how to use this VM, based on standard Ubuntu 20.04 and Ros Kinetic installations. Fur students working with macOS, an alternative installation method based on [RoboStack](https://robostack.github.io/) is proposed [in the repository](Practicals/macOS/Robostack_on_macOS.pdf)

The students have access to about 30 [turtlebot 3 burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/). Each robot emits a wifi hotspot with a SSID ***turtlebotX_wifi***, with $X$ the number of the robot sticked on the higher plate. The way to interact with the robot through ROS (wifi connection, network parameterization of the VM, etc.) is indicated in the [turtlebot_guide](Practicals/Turtlebot_guide/turtlebot_guide.pdf) document.

---

To contact us: (add sorbonne-universite to our email to reach us!)
- Sylvain Argentieri <sylvain.argentieri@>
- Fabien Vérité <fabien.verite@>
- Ludovic Saint Bauzel <ludovic.saint-bauzel@>