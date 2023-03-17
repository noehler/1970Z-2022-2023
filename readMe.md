For Judges and those interested:
Our code is organized into several different classes. The two main classes are:
- sensing class- This is defined in "include/robotConfig.h". In in this class the position calculations and sensor input are done.
- motor Control class- This is defined in "include/motorControl.h".  This class deals with various motorcontrollers and final calculations(like goal speed in inches per second to rpm needed). 

To find our macros used for autons you can look in "src/Autons/autonSetup.cpp"
The main thread takes place in "src/main.cpp". From here all other threads are started. 



For future students:
Thank you for looking at our code! Please feel free to look through and take inspiration from our code, we worked really hard and would like to save you some time in the future. 

If you end up using our code or taking inspiriation, please cite our team and github (judges look for these things and it is part of the notebooking rubric).

- I believe the most useful section for most years will be our special way of tracking odometry. Rather than assuming that the robot traveled in a straight line from the last point it was at, which is often not the case, we approximate an arcing path, which I have found to be far more reliable as long as all of the constants are defined properly. You can find this is the function odometry() in the public section of sensing_t inside "include/robotConfig.h".

- Also another function which could be useful is the SSOSTTT() function. This is a function that is used to account for robot velocity and other factors to properly aim a turret and spin up a flywheel. This can be found in SSOSTTT() in public of motorControl_t in "include/motorcontrol.h"



Final Note:
We have everything defined in inches(sorry, but we find this to be the best measurement system for the vex field as it is designed around these systems), so if you work in meters you will need to adapt these equations for your own purposes. For help in these you can view our notebook at [insert link here].
