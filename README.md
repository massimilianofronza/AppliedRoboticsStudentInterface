# Laboratory of Applied Robotics Student Interface
Package used by students to complete the assignment of the course. The scope of this project was designing a way to navigate a robot through an arena, recovering victims in a fixed order and reaching the exit gate, all without touching the obstacles.
The first part involved the processing of the raw image coming from a camera situated over the arena, with calibration, rectification and unwarping, developed with OpenCV.  Knowing the colors of the entities, the polygons composing the victims were first highlighted, then identified via template matching, and finally ordered. Obstacles were virtually enlarged via a fixed offset to avoid collisions due to the usage of the Rapidly Exploring Random Tree algorithm. To define the set of possible movements of this non-holonomic robot, we created a series of Dubin's paths. Finally, we computed the path for the robot to follow with the RRTstar algorithm from the Open Motion Planning Library.

### A side view of the arena in the 3D environment of development

<p align="left">
  <img width="600" src=https://github.com/massimilianofronza/AppliedRoboticsStudentInterface/assets/38779834/e4eb45e9-8dfc-427c-b78a-6db667ddda73>
</p>

### The virtual representation of the maze, with the path defined and ready to be passed to the controller

<p align="left">
  <img width="600" src=https://github.com/massimilianofronza/AppliedRoboticsStudentInterface/assets/38779834/2cf4e6c7-0a0e-4703-be58-f92a368352a0>
</p>
