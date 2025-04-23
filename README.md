# hand_modeling_toolbox

## descriptions

## classes
### Finger
The Finger object represents a serial robot connected by a revolute joint.
For validation, the Finger object has an avatar model generated in Robotic
System Toolbox from Matlab. After defining the Finger object, use update_rst_model.m to generate an rst model (`Finger.update_rst_model`). 

#### How to create a Finger object: 

+ finger = Finger(finger_name, finger_type, link_len_vector)
    * finger_name: [char] 
    * finger_type: 
        * 'RRRR': general finger with four joints(Ab/duction, MCP, PIP , DIP);
        * 'RRRp': planar finger with three joints(MCP, PIP, DIP; ABD joint is fixed);
        * 'RRRs': spatial finger with three joints(Ab/duction, MCP, PIP; DIP joint is fixed);
    * link_len_vector: vector of the length of the links that match the number the finger_type (mdh: a);
+ finger = Finger(finger_name, 'mdh',mdh_struct);
   + finger_name: [char] 
   + mdh_struct: structure contains mdh parameters; use mdh_matrix_to_struct(mdh_matrix,1) to derive
+ create_finger_random(finger_name, finger_dof)
   + Create a random finger with name and DoF. Normally used for initializing a finger with random parameters and then modifying the mdh parameters.

#### Coordinates of a Finger object 
For a Finger object with *nj* joints, there are *nj + 3* frames
+ Coord. World (w): world coordinate. symmetrical for all classes
+ Coord. base (b):  original coordinate for the **Class Finger**. Transformation information regarding w: *Finger.w_p_base* and *Finger.w_R_base*
+ Coord. of each links(joints)
+ Coord. of End-Effector 

#### Finger.list_links
this property saves all the Link objects of the Finger
For a Finger with *nj* DoF, there is *nj+1* Link objects in total:
+ Finger.base: the base of the finger, including kinematic and dynamic parameters
+ Finger.list_links: each link has a Link object listed in the Finger.list_links

#### Kinematic parameters (saved as a structure with dimension [nj+1,4])
Note: theta parameters are set to 0 initially and considered as q
+ Finger.mdh: alpha, a, theta (q), d 
+ Finger.mdha_all: same as Finger.mdh (current version)
+ Finger.mdh_ori: alpha, a, theta (without q), d 

#### Dynamic parameters (Finger.par_dyn_f)
+ g: [3,1] gravity vector ([0;0;-9.81] by default)
+ mall_all: [nl+1,1] mass value of base and links
+ inertia_all: [6,nl+1]
+ com_all: [3,nl+1]

#### Base/Link functions
+ w_T_base:  transformation matrix of Finger base to Finger world frame
+ w_T_base_inhand:  transformation matrix of Finger base to Hand world frame


#### Kinematics:
+ position (test script *test_finger.m*)
    + get_W_T_B: get base transformation matrix
    + get_p_all_links: get cartesian position of all link frames
    + get_T_all_links: get transformation matrix of all link frames    
+ Jacobian (test script *test_finger.m*)
    + Jacobian_geom_b_end: geometric Jacobian of End Effector represented in the base frame
    + Jacobian_geom_w_end: geometric Jacobian of End Effector represented in the world frame
+ FK (test script *test_finger.m*)
    + update_finger(varargin): update Finger object with given joint configuration
+ IK: (test script *test_ik.m*)
    + IK_par: class of ik parameters for solvers
    + invkin_numeric: Newton-Raphson algorithm
    + invkin_numeric_LM: Levenberg-marquart algorithm

#### Symbolic computation of dynamics
Check test script *test_finger_symbolic.m*
+ *Finger.invdyn_ne_w_end_sym*: 
Fixed-base inverse dynamic of the Finger with the end effector interaction to the environment with respect to the world frame.
Link to *invdyn_ne_mdh_sym.m*
+ *Finger.invdyn_ne_xq_fb_all_fext_sym*: 
Floating-based inverse dynamic of the Finger. Link to *invdyn_ne_xq_mdh_all_fext_sym.m*
+ *Finger.invdyn_ne_xq_fb_wt_fext_sub_sym*: Floating-based inverse dynamic with individual terms. Link to *invdyn_ne_xq_mdh_wt_fext_sub_sym.m*
+ *Finger.invdyn_ne_xq_fb_wt_fext_sub_sym_par*: Floating-based inverse dynamic with individual terms calculated with *parfor*.


#### Finger.kin_T_par
Kinematics
+ get_T_all_links
+ get_T_ee_w
+ get_b_T_all_links
+ Jacobian_geom_w_end: 
+ Jacobian_geom_b_end: 
+ Jacobian_analytic_b_end: not compatible
+ Jacobian_geom_b_contact: not compatible
Dynamics
+ invdyn_ne_w_end: 
+ invdyn_ne_w_end_T: individual version for kin_T_par
+ invdyn_ne_xq_fb_all_fext: not compatible
+ invdyn_ne_xq_fb_wt_fext_sub: not compatible


















