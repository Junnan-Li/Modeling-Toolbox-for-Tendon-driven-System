% Calculate forward kinematics (homogenous transformation matrices) for fixed-base
% exoprB01
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [7x1]
%   Generalized joint coordinates (joint angles)
% pkin [4x1]
%   kinematic parameters (e.g. lengths of the links)
% 
% Output:
% T_c_mdh [4x4x(7+1)]
%   homogenous transformation matrices for each (body) frame (MDH)
%   1:  mdh base (link 0) -> mdh base link 0 (unit matrix, no information)
%   ...
%   8:  mdh base (link 0) -> mdh frame (8-1), link (8-1)
%   ...
%   7+1:  mdh base (link 0) -> mdh frame (7)

% Quelle: IRT-Maple-Repo
% Datum: 2020-02-05 19:50
% Revision: fbe1615c2d83fd5f15c195a10cc896a653daee7e
% (C) Institut für Regelungstechnik, Universität Hannover

function T_c_mdh = exoprB01_fkine_fixb_rotmat_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
assert(isa(qJ,'double') && isreal(qJ) && all(size(qJ) == [7 1]), ...
  'exoprB01_fkine_fixb_rotmat_mdh_sym_varpar: qJ has to be [7x1] double');
assert(isa(pkin,'double') && isreal(pkin) && all(size(pkin) == [4 1]), ...
  'exoprB01_fkine_fixb_rotmat_mdh_sym_varpar: pkin has to be [4x1] double');
%% Variable Initialization

qJ1s = qJ(1);
qJ2s = qJ(2);
qJ3s = qJ(3);
qJ4s = qJ(4);
qJ5s = qJ(5);
qJ6s = qJ(6);
qJ7s = qJ(7);
rxs_base=0;
rys_base=0;
rzs_base=0;

a4 = pkin(1);
a5 = pkin(2);
d4 = pkin(3);
d5 = pkin(4);

%% Symbolic Calculation
% From fkine_mdh_floatb_twist_rotmat_matlab.m
% OptimizationMode: 1
% StartTime: 2020-02-05 16:57:31
% EndTime: 2020-02-05 16:57:31
% DurationCPUTime: 0.10s
% Computational Cost: add. (279->58), mult. (702->74), div. (0->0), fcn. (990->14), ass. (0->179)
unknown=NaN(32,4);
t1 = sin(qJ1s);
t2 = cos(qJ1s);
t3 = sin(qJ2s);
t4 = t1 * t3;
t5 = cos(qJ2s);
t6 = t1 * t5;
t7 = t2 * t3;
t8 = t2 * t5;
t9 = cos(qJ3s);
t11 = sin(qJ3s);
t13 = t2 * t11 + t4 * t9;
t16 = -t4 * t11 + t2 * t9;
t17 = t5 * t9;
t18 = t5 * t11;
t21 = -t1 * t11 + t7 * t9;
t24 = -t1 * t9 - t7 * t11;
t25 = cos(qJ4s);
t27 = sin(qJ4s);
t29 = -t13 * t25 - t16 * t27;
t32 = t13 * t27 - t16 * t25;
t33 = t13 * a4;
t34 = t6 * d4;
t38 = t17 * t25 - t18 * t27;
t41 = -t17 * t27 - t18 * t25;
t42 = t17 * a4;
t43 = t3 * d4;
t47 = -t21 * t25 - t24 * t27;
t50 = t21 * t27 - t24 * t25;
t51 = t21 * a4;
t52 = t8 * d4;
t54 = sin(qJ5s);
t56 = cos(qJ5s);
t58 = -t29 * t54 + t6 * t56;
t61 = -t29 * t56 - t6 * t54;
t64 = t29 * a5 - t32 * d5 + t33 + t34 + rxs_base;
t67 = t3 * t56 - t38 * t54;
t70 = -t3 * t54 - t38 * t56;
t73 = t38 * a5 - t41 * d5 - t42 + t43 + rys_base;
t76 = -t47 * t54 + t8 * t56;
t79 = -t47 * t56 - t8 * t54;
t82 = t47 * a5 - t50 * d5 + t51 + t52 + rzs_base;
t83 = sin(qJ6s);
t85 = cos(qJ6s);
t87 = -t32 * t85 - t58 * t83;
t90 = t32 * t83 - t58 * t85;
t93 = -t41 * t85 - t67 * t83;
t96 = t41 * t83 - t67 * t85;
t99 = -t50 * t85 - t76 * t83;
t102 = t50 * t83 - t76 * t85;
t103 = cos(qJ7s);
t105 = sin(qJ7s);
unknown(1,1) = 1;
unknown(1,2) = 0;
unknown(1,3) = 0;
unknown(1,4) = rxs_base;
unknown(2,1) = 0;
unknown(2,2) = 1;
unknown(2,3) = 0;
unknown(2,4) = rys_base;
unknown(3,1) = 0;
unknown(3,2) = 0;
unknown(3,3) = 1;
unknown(3,4) = rzs_base;
unknown(4,1) = 0;
unknown(4,2) = 0;
unknown(4,3) = 0;
unknown(4,4) = 1;
unknown(5,1) = -t1;
unknown(5,2) = -t2;
unknown(5,3) = 0;
unknown(5,4) = rxs_base;
unknown(6,1) = 0;
unknown(6,2) = 0;
unknown(6,3) = 1;
unknown(6,4) = rys_base;
unknown(7,1) = -t2;
unknown(7,2) = t1;
unknown(7,3) = 0;
unknown(7,4) = rzs_base;
unknown(8,1) = 0;
unknown(8,2) = 0;
unknown(8,3) = 0;
unknown(8,4) = 1;
unknown(9,1) = t4;
unknown(9,2) = t6;
unknown(9,3) = -t2;
unknown(9,4) = rxs_base;
unknown(10,1) = -t5;
unknown(10,2) = t3;
unknown(10,3) = 0;
unknown(10,4) = rys_base;
unknown(11,1) = t7;
unknown(11,2) = t8;
unknown(11,3) = t1;
unknown(11,4) = rzs_base;
unknown(12,1) = 0;
unknown(12,2) = 0;
unknown(12,3) = 0;
unknown(12,4) = 1;
unknown(13,1) = t13;
unknown(13,2) = t16;
unknown(13,3) = t6;
unknown(13,4) = rxs_base;
unknown(14,1) = -t17;
unknown(14,2) = t18;
unknown(14,3) = t3;
unknown(14,4) = rys_base;
unknown(15,1) = t21;
unknown(15,2) = t24;
unknown(15,3) = t8;
unknown(15,4) = rzs_base;
unknown(16,1) = 0;
unknown(16,2) = 0;
unknown(16,3) = 0;
unknown(16,4) = 1;
unknown(17,1) = t29;
unknown(17,2) = t32;
unknown(17,3) = t6;
unknown(17,4) = (t33 + t34 + rxs_base);
unknown(18,1) = t38;
unknown(18,2) = t41;
unknown(18,3) = t3;
unknown(18,4) = (-t42 + t43 + rys_base);
unknown(19,1) = t47;
unknown(19,2) = t50;
unknown(19,3) = t8;
unknown(19,4) = (t51 + t52 + rzs_base);
unknown(20,1) = 0;
unknown(20,2) = 0;
unknown(20,3) = 0;
unknown(20,4) = 1;
unknown(21,1) = t58;
unknown(21,2) = t61;
unknown(21,3) = -t32;
unknown(21,4) = t64;
unknown(22,1) = t67;
unknown(22,2) = t70;
unknown(22,3) = -t41;
unknown(22,4) = t73;
unknown(23,1) = t76;
unknown(23,2) = t79;
unknown(23,3) = -t50;
unknown(23,4) = t82;
unknown(24,1) = 0;
unknown(24,2) = 0;
unknown(24,3) = 0;
unknown(24,4) = 1;
unknown(25,1) = t87;
unknown(25,2) = t90;
unknown(25,3) = -t61;
unknown(25,4) = t64;
unknown(26,1) = t93;
unknown(26,2) = t96;
unknown(26,3) = -t70;
unknown(26,4) = t73;
unknown(27,1) = t99;
unknown(27,2) = t102;
unknown(27,3) = -t79;
unknown(27,4) = t82;
unknown(28,1) = 0;
unknown(28,2) = 0;
unknown(28,3) = 0;
unknown(28,4) = 1;
unknown(29,1) = (t87 * t103 + t61 * t105);
unknown(29,2) = (t61 * t103 - t87 * t105);
unknown(29,3) = t90;
unknown(29,4) = t64;
unknown(30,1) = (t93 * t103 + t70 * t105);
unknown(30,2) = (t70 * t103 - t93 * t105);
unknown(30,3) = t96;
unknown(30,4) = t73;
unknown(31,1) = (t99 * t103 + t79 * t105);
unknown(31,2) = (t79 * t103 - t99 * t105);
unknown(31,3) = t102;
unknown(31,4) = t82;
unknown(32,1) = 0;
unknown(32,2) = 0;
unknown(32,3) = 0;
unknown(32,4) = 1;
unknown = unknown; %#ok<ASGSL> für automatische Verarbeitung
T_ges = unknown ;
%% Postprocessing: Reshape Output
% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)
T_c_mdh = NaN(4,4,7+1);
for i = 1:7+1
  T_c_mdh(:,:,i) = T_ges((i-1)*4+1 : 4*i, :);
end
