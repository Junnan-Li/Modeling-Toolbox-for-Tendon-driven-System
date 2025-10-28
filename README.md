# hand_modeling_toolbox

## descriptions
This toolbox provides a mathematic framework and tools to simulate the kinematics and dynamics for the tendon-driven robotic hand and muscle-driven human hand (musculoskeletal model).
It is able to simulate serial articulated structure robot, or hybrid structure that multiple serial articulated robots connected to one serial robot, such as five fingers connect to arm/wrist.  
Beyond joint space, muscle/tendon class is implemented to the model as actuators to simulate musculotendon or tendon-driven robots.  


This toolbox is developed initially for dynamic modeling the tendon-driven hand & arm (both robotic and human hands) in Matlab and Simulink for identification and realtime control.
But it is not limited to this. It is able to model the system which does not have complicated structure (parallel or complex hybrid). 

The toolbox is functional to use but still under development for further features. 
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


## Classes
### Finger
The **Finger** class is defined to modal a serial robot connected by revolute joints.
The according Robotic System Toolbox model is generated using `Finger.update_rst_model`.
### important properties
+ `kin_use_T` : if use transformation matrix to defined kinematics (it will override mdh parameters)
+ `nj`:  number of joints
+ `nl`:  number of links
+ `nmus`:  number of muscles
+ `nvia`:  number of viapoints
+ `list_...`:  list of associated objects defined in the Finger object
+ `base`: a link object acts as the base of Finger object

+ `index_finger_inhand`: the index of this Finger object in its parent Hand object
+ `w_T_base`: tranformation matrix between base link and world frame
+ `w_T_base_inhand`: updated tranformation matrix between base link and world frame once it is implemented in the Hand object
+ `rst_model`: Robotic System Toolbox model
+ `mdh_ori`: mdh parameters without q in $\theta$ column
+ `mdh_all`: mdh parameters with q in $\theta$ column
+ `par_dyn_f`: dynamic parameters including 
    + [nj+1,1] mass vector
    + [3,nj+1] centor of mass
    + [6,nj+1] inetia
    + [3,1] gravity vector
+ `par_T_link`: [4x(nl+1),4] transformation matrix between each link to replace mdh parameters (include end)

#### How to create a Finger object: 
+ `finger = Finger(finger_name, 'mdh',mdh_struct)`;
create a finger object with name and mdh parameter structure as input 
+ `create_finger_random(finger_name, finger_dof)`;
Create a random finger with name and number of DoFs. Normally used for initializing a finger with random parameters and then modifying the mdh parameters.

#### Coordinates of a Finger object 
For a Finger object with *nj* joints, there are *nj + 3* frames
+ Coord. World (w): world coordinate. symmetrical for all classes
+ Coord. base (b):  original coordinate for the **Class Finger**. Transformation information regarding w: *Finger.w_p_base* and *Finger.w_R_base*
+ Coord. of each links (joints)
+ Coord. of End-Effector (e) 

#### Finger.list_links
this property saves all the Link objects of the Finger
For a Finger with *nj* DoF, there is *nj+1* Link objects in total:
+ `Finger.base`: the base of the finger, including kinematic and dynamic parameters
+ `Finger.list_links`: each link has a Link object listed in the Finger.list_links

#### Kinematics:
position (test script **test_finger.m**)
+ `Finger.get_W_T_B`: get base transformation matrix
+ `Finger.get_p_all_links`: get cartesian position of all link frames
+ `Finger.get_T_all_links`: get transformation matrix of all link frames
    
Jacobian (test script **test_finger.m**) 
+ `Finger.Jacobian_geom_b_end`: geometric Jacobian of End Effector represented in the base frame
+ `Finger.Jacobian_geom_w_end`: geometric Jacobian of End Effector represented in the world frame

Forward Kinematics: (test script **test_finger.m**)
+ `Finger.update_finger(varargin)`: update Finger object with given joint configuration

Inverse Kinematics: (test script **test_ik.m**)

+ IK_par: class of ik parameters for solvers
+ `Finger.invkin_numeric`: Newton-Raphson algorithm
+ `Finger.invkin_numeric_LM`: Levenberg-marquart algorithm

Kinematic definition using `Finger.kin_use_T`
+ check test script **test_finger_par_T.m**

#### Dynamics:
+ `Finger.invdyn_ne_w_end`: inverse dynamic Newton-Euler method in world frame with end-effector force
+ `Finger.invdyn_ne_w_end_T`: inverse dynamic Newton-Euler method in world frame with end-effector force whose kinematic is defined using `Finger.par_T_link`
+ `Finger.invdyn_ne_xq_fb_all_fext`: specific version for floating base with additional 6 system state for base

#### Symbolic computation of dynamics
Check test script *test_finger_symbolic.m*
+ `Finger.sym_T_all_links_from_q`: generate symbolic function to calculate transformation matrix of each link with q as input. for Simulink usage. 
+ `Finger.invdyn_ne_w_end_sym`: fixed-base inverse dynamic of the Finger with the end effector interaction to the environment with respect to the world frame.
Link to *invdyn_ne_mdh_sym.m*
+ `Finger.invdyn_ne_xq_fb_all_fext_sym`: 
Floating-based inverse dynamic of the Finger. Link to `invdyn_ne_xq_mdh_all_fext_sym.m`
+ `Finger.invdyn_ne_xq_fb_wt_fext_sub_sym`: Floating-based inverse dynamic with individual terms. Link to *invdyn_ne_xq_mdh_wt_fext_sub_sym.m*
+ `Finger.invdyn_ne_xq_fb_wt_fext_sub_sym_par`: Floating-based inverse dynamic with individual terms calculated with *parfor*.

#### Visualization
+ `parstr = Finger.plot_parameter_init`: parameters for plot functions
+ `Finger.plot_finger()`: plot finger and rotation axes. optional parameters for plot
+ `Finger.plot_com()`: plot center of mass
+ `Finger.plot_viapoints()`: plot viapoints
+ `Finger.plot_muscles()`: plot muscles
+ `Finger.plot_obstacles()`: plot obstacles


### Hand
The **Hand** object integrates several Finger objects together and stores into `Hand.base` and `Hand.fingers`. 
The kinematic and dynamic computation is adapted accordingly. 
#### important properties
+ `Hand.base` : a stack of Finger objects which are serially articulated, the end-effector frame of the last base object is taken as the world frame of the next base object
+ `Hand.fingers`:  a stack of Finger objects which are parallelly connected to the last Finger object in `Hand.base`
+ `nb`:  number of Finger objects in `Hand.base`
+ `nf`:  number of Finger objects in `Hand.fingers`
+ `nj`:  number of total joints
+ `njb`:  number of joints in `Hand.base`
+ `njf`:  number of joints in `Hand.fingers`
+ `nmus`:  number of muscles of all Finger objects
+ `nvia`:  number of viapoints of all Finger objects
+ `list_...`:  list of associated objects of all Finger objects
+ `index_q_b`: [njb,2] start and end index of base joint in q
+ `index_q_f`: [njf,2] start and end index of base joint in q
+ `sim`: reformulated model parameters for Simulink applications
+ `par_dyn_h`: dynamic parameters of all bases and fingers

#### create Hand model
+ `hand_obj = create_hand_random(hand_name, varargin )`

create a hand object with random mdh, base configuration, and dynamic parameters. 
For example, create_hand_random("hand_example", [2,3,4,3] ): create a hand with 2
bases, each of which has 3 DoFs, and 4 fingers, each of which has 3 DoFs

+ `hand_obj = Hand('Hand_name')` + `hand_obj.add_base(base_1)` + `hand_obj.add_finger(finger_1)`

initialize a Hand object and manually add pre-defined base and fingers to the Hand object 

#### Kinematics 
Position:
+ `Hand.get_w_T_ee_all`: get the T matrix of the all fingers
+ `Hand.get_w_T_all`: get the T matrix of the frames

Jacobian:
+ `Hand.Jacobian_geom_w_all_fingers`: geometrical Jacobian of all finger EE represented in world frame
+ `Hand.Jacobian_geom_w_vp`: geometrical Jacobian of a viapoint represented in world frame

Forward Kinematics:
+ `Hand.update_hand(q)`: update hand kinematic with given q 

Inverse Kinematics: 

+ `Hand.invkin_numeric_LM`: calculate ik with given finger EE position using Levenberg Marquardt algorithm
+ `Hand.invkin_numeric_LM_vp`: calculate ik with given viapoint position using Levenberg Marquardt algorithm

Muscle Path with obstacle:

+ `Hand.cal_hand_Muscle_l_J_Garner`: calculate muscle path and moment arm [4]

#### Dynamics
Forward Dynamic
+ `Hand.fordyn_ne_hand_w_end`

Inverse Dynamic
+ `Hand.invdyn_ne_hand_w_end`

#### Symbolic computation of dynamics


#### Simulink implementation
+ `Hand.create_sim_functions_hand`: 
create functions for simulink application with individual Hand model parameters for static memory allocation


#### Visualization
+ `parstr = Hand.plot_parameter_init`: parameters for plot functions
+ `Hand.plot_finger()`: plot finger and rotation axes. optional parameters for plot
+ `Hand.plot_com()`: plot center of mass
+ `Hand.plot_viapoints()`: plot viapoints
+ `Hand.plot_muscles()`: plot muscles
+ `Hand.plot_obstacles()`: plot obstacles


## Test scripts
Test scripts for testing functions while development stored in the **/test** folder
+ **test_finger.m**
+ **test_finger_par_T.m**
+ **test_hand_dynamics.m**
+ **test_muscle_viapoint.m**
+ **test_simulink_functions.m**

...


## Example scripts
Example scripts for modeling and simulations stored in the **/examples** folder


# Reference

[1] Li, Junnan, Amartya Ganguly, Luis FC Figueredo, and Sami Haddadin. "The fingertip manipulability assessment of Tendon-driven multi-fingered hands." IEEE Robotics and Automation Letters 9, no. 3 (2024): 2726-2733.

[2] Li, Junnan, Lingyun Chen, Johannes Ringwald, Edmundo Pozo Fortunić, Amartya Ganguly, and Sami Haddadin. "Identification and validation of the dynamic model of a tendon-driven anthropomorphic finger." In 2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pp. 8330-8337. IEEE, 2024.

[3] Li, Junnan, Amartya Ganguly, Luis FC Figueredo, and Sami Haddadin. "Tendon to Object Space: Evaluation of Anthropomorphic Finger for Human-Like Performance." In International Workshop on Human-Friendly Robotics, pp. 196-212. Cham: Springer Nature Switzerland, 2023.

[4] Garner, Brian A., and Marcus G. Pandy. "The obstacle-set method for representing muscle paths in musculoskeletal models." Computer methods in biomechanics and biomedical engineering 3.1 (2000): 1-30.











