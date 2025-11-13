# Modeling-Toolbox-for-Tendon-driven-System

## descriptions
This toolbox provides a mathematic framework and tools to simulate the kinematics and dynamics for the tendon-driven robotic hand and muscle-driven human hand (musculoskeletal model).
It is able to simulate serial articulated structure robot, or hybrid structure that multiple serial articulated robots connected to one serial robot, such as five fingers connect to arm/wrist.  
Beyond joint space, muscle/tendon class is implemented to the model as actuators to simulate musculotendon or tendon-driven robots.  


This toolbox is developed initially for dynamic modeling the tendon-driven hand & arm (both robotic and human hands) in Matlab and Simulink for identification and realtime control.
But it is not limited to this. It is able to model the system which does not have complicated structure (parallel or complex hybrid). 

The toolbox is functional to use but still under development for further features.   
For a detailed documentation please check [Documentation](./doc/Documentation.md)


## features
+ kinematic definition using MDH parameters or transformation matrix 
+ dynamic computation with different methods, such as Lagrangian and Newton-Euler. 
+ dynamic computation with floating base by adding 6 DoFs in the system state. 
+ generalized version of kinematic and dynamic functions
+ a Robotic System Toolbox model is converted from this model for testing of implemented functions. 
+ symbolic function generation to compute each dynamic term 
+ muscle/tendon path computation and associated Jacobian [4]
+ implementation of inverse kinematic algorithms (Newton-raphson, Levenberg-Marquardt) with multiple target points/markers. 
+ implementation of solvers (Euler, Runge-Kutta) for MATLAB simulation.  
+ a masked Simulink library with model object as input for realtime Simulink applications.

## structure of classes
There are several concatenated classes defined in this toolbox
+ **Hand**: highest level of class integrates all other class objects
+ **Finger**: modeled as serial robot, containing Links and Joints objects and optional muscles
+ **Links**: rigid body with defined dynamic parameters and updating kinematic information while integrated into a Finger object. 
+ **Joints**: refers to joint with joint limit information and index when integrated into a Finger object.
+ **ViaPoint** and **Contacts** : defined in Links object via Finger or Hand functions for muscles path.
+ **Muscle** and **Tendon**: can be defined within Finger or Hand via a set of ViaPoints and Obstacles as path for moment arm computation.
+ **Obstacles** and subclasses like **Cylinder_Obs** : defined in Links object for muscle path 
 
A **Hand** object consists a set of serial articulated **Finger** objects listed in **Hand.base** 
and another set of independent **Finger** objects connected to the End-Effector of the last object in **Hand.base**.  
For detailed information, please check test or example scripts and **doc/Documentation.md**



## Basic usage
### How to create a Finger object: 
+ `finger = Finger(finger_name, 'mdh',mdh_struct)`;
create a finger object with name and mdh parameter structure as input 
+ `create_finger_random(finger_name, finger_dof)`;
Create a random finger with name and number of DoFs. Normally used for initializing a finger with random parameters and then modifying the mdh parameters.

### create Hand model
+ `hand_obj = create_hand_random(hand_name, varargin )`

create a hand object with random mdh, base configuration, and dynamic parameters. 
For example, create_hand_random("hand_example", [2,3,4,3] ): create a hand with 2
bases, each of which has 3 DoFs, and 4 fingers, each of which has 3 DoFs

+ `hand_obj = Hand('Hand_name')` + `hand_obj.add_base(base_1)` + `hand_obj.add_finger(finger_1)`

initialize a Hand object and manually add pre-defined base and fingers to the Hand object 
## Examples
`create_Dexmart_index.m`:
create a model of Index finger of the Dexmart Hand.   
`create_finger_random(finger_name, finger_dof)`: create a random finger object with desired DoFs   
`create_finger_random_par_T(finger_name, finger_dof)`: create a random finger object using transformation matrix 
for defining kinemtics    
`create_hand_random(hand_name, varargin)`: create a random Hand object   
`create_shadow_hand.m`: create a dynamic model of the **Shadow Hand** with customized 2N full actuated tendon routing   
`example_shadowhand_simulation.m`: a simple dynamic simulation of the **Shadow Hand** with 2N tendon routing for a pose reaching task.
A impedance joint torque control is implemented and the required tendon forces are computed.    
`lugre_example.m`: example of **LuGre** friction model for tendon-driven system.   
`/Mojtaba_hand_model`: generate a kinematic musculoskeletal model from literature [4] of a cadaver hand    
`/Franka`: an example of dynamic **Franka** robot model and associated symbolic functions to calculate individual 
dynamic terms (Mass matrix, Coriolis, and Gravity vectors) for other applications
`/ARMS_hand`: transfer a OpenSim hand/wrist model [5] (not finished)

## Test scripts
Test scripts for testing functions while development stored in the **/test** folder
+ `test_finger.m`: A test script of Finger object functions (FK, Jacobian, ID, IK) compared to the 
Robotic System Toolbox model `Finger.rst_model`

+ `test_finger_par_T.m`: Another test script for Finger object whose kinematic is defined using 
Transformation matrix `Finger.par_T_link` with `Finger.kin_use_T` on.

+ `test_finger_symbolic.m`: test of symbolic function generation regarding ID (`Finger.invdyn_..._sym`)
+ `test_hand_dynamics.m`: test `Hand.invdyn_ne_hand_w_end` to `rst_model.inverseDynamics(q,qd,qdd)` of Robotic System Toolbox model
+ `test_hand.m` : basic tests of Hand object 
+ `test_hand_RST.m`: test of generating rst model for Hand object
+ `test_IK.m`: compare inverse kinematic algorithms 
+ `test_viapoint.m`: example of defining muscles and viapoints
+ `test_simulink_functions.m`: test `Hand.create_sim_functions_hand` that generates .m functions that can be applied in Simulink applications 
avoiding problem of calling Class function as Sfunction.


## Other information
Toolbox is under development, any interest or question about this toolbox please contact Junnan Li (<junnan.li@tum.de>).



# Reference

[1] Li, Junnan, Amartya Ganguly, Luis FC Figueredo, and Sami Haddadin. "The fingertip manipulability assessment of Tendon-driven multi-fingered hands." IEEE Robotics and Automation Letters 9, no. 3 (2024): 2726-2733.   

[2] Li, Junnan, Lingyun Chen, Johannes Ringwald, Edmundo Pozo Fortunić, Amartya Ganguly, and Sami Haddadin. "Identification and validation of the dynamic model of a tendon-driven anthropomorphic finger." In 2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pp. 8330-8337. IEEE, 2024.  

[3] Li, Junnan, Amartya Ganguly, Luis FC Figueredo, and Sami Haddadin. "Tendon to Object Space: Evaluation of Anthropomorphic Finger for Human-Like Performance." In International Workshop on Human-Friendly Robotics, pp. 196-212. Cham: Springer Nature Switzerland, 2023.

[4] M. Mirakhorlo et al., "Anatomical parameters for musculoskeletal modeling of the hand and wrist," International Biomechanics, vol. 3, no. 1, pp. 1–10, 2016, doi: 10.1080/23335432.2016.1191373.

[5] McFarland, Daniel C., et al. "A musculoskeletal model of the hand and wrist capable of simulating functional tasks." IEEE transactions on biomedical engineering 70.5 (2022): 1424-1435.







