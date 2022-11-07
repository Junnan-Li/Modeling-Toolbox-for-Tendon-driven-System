




%% build tree
finger = rigidBodyTree;
finger.DataFormat = 'column';
finger.Gravity = [0 0 -9.81];



% create body
body1 = rigidBody('phl');
body1.Mass = 1;
body1.CenterOfMass = [0 0 0];
body1.Inertia = [1 1 1 0 0 0];

body2 = rigidBody('pphl');
body3 = rigidBody('mphl');
body4 = rigidBody('dphl');
body5 = rigidBody('ft');

% collision object
collisionObj = collisionCylinder(0.02,0.25);

% create joints
wrist = rigidBodyJoint('wrist');
% wrist.HomePosition = pi/2;
tform = eul2tform([0 0 -pi/2]); % User defined
setFixedTransform(wrist,tform);
body1.Joint = wrist;
addBody(finger,body1,finger.BaseName)


MCP = rigidBodyJoint('MCP','revolute');
MCP.HomePosition = pi/8;
tform = trvec2tform([1, 0, 0]); % User defined
setFixedTransform(MCP,tform);
body2.Joint = MCP;
addBody(finger,body2,'phl')


PIP = rigidBodyJoint('PIP','revolute');
PIP.HomePosition = pi/8;
tform = trvec2tform([0.7, 0, 0]); % User defined
setFixedTransform(PIP,tform);
body3.Joint = PIP;
addBody(finger,body3,'pphl')


DIP = rigidBodyJoint('DIP','revolute');
DIP.HomePosition = pi/8;
tform = trvec2tform([0.5, 0, 0]); % User defined
setFixedTransform(DIP,tform);
body4.Joint = DIP;
addBody(finger,body4,'mphl')

ftjoint = rigidBodyJoint('ftjoint');
tform = trvec2tform([0.3, 0, 0]); % User defined
setFixedTransform(ftjoint,tform);
body5.Joint = ftjoint;
addBody(finger,body5,'dphl')


T = trvec2tform([collisionObj.Length/2 0 0]) * eul2tform([0 pi/2 0], 'XYZ');
for i = 1:finger.NumBodies-1
    dimension_colobj = tform2trvec(finger.Bodies{i+1}.Joint.JointToParentTransform)
    collisionObj = collisionCylinder(0.02,dimension_colobj(1));
    T = trvec2tform([collisionObj.Length/2 0 0]) * eul2tform([0 pi/2 0], 'XYZ');
    
    addCollision(finger.Bodies{i},collisionObj,T);
    
end

finger.checkCollision([1,1,1]');

show(finger,finger.homeConfiguration,'Collisions','on','Visuals','off');
showdetails(finger)

%% kinematic

q = homeConfiguration(finger);
% q.JointPosition

q0 = [0.3,0,0]';
show(finger,q0,'Collisions','on','Visuals','off');
transform = getTransform(finger,q,'ft');
Jac = finger.geometricJacobian(q0, 'ft')



%% Dynamic

q = [.2,.5,.2]';
q_dot = [.1,.2,.3]';

M = massMatrix(finger,q)
C = velocityProduct(finger,q,q_dot)
G = gravityTorque(finger,q)

% jointTorq = inverseDynamics(robot,configuration,jointVel,jointAccel,fext)


