clc; clear; close all;

% Define the robot as a rigid body tree
robot = rigidBodyTree('DataFormat', 'column');

% Define link lengths (meters)
L1 = 0.5; % Base to joint 2
L2 = 0.5; % Joint 2 to joint 3
L3 = 0.5; % Joint 3 to end-effector

%% Joint 1 (Base Rotation - rotates the entire arm)
body1 = rigidBody('body1');
joint1 = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint1, trvec2tform([0, 0, 0])); % Base is at the origin
body1.Joint = joint1;
addBody(robot, body1, 'base');

%% Joint 2 (Moves up/down, should extend along X)
body2 = rigidBody('body2');
joint2 = rigidBodyJoint('joint2', 'revolute');
setFixedTransform(joint2, trvec2tform([0, 0, 0.2]) * axang2tform([1 0 0 pi/2])); 
% Moved L1 along X and rotated to move in the XZ plane
body2.Joint = joint2;
addBody(robot, body2, 'body1'); % Attach to Joint 1

%% Joint 3 (Also moves up/down, positioned further along X)
body3 = rigidBody('body3');
joint3 = rigidBodyJoint('joint3', 'revolute');
setFixedTransform(joint3, trvec2tform([L2, 0.3, 0]) * axang2tform([0 0 1 pi/2])); 
body3.Joint = joint3;
addBody(robot, body3, 'body2'); % Attach to Joint 2

%% End-Effector
endEffector = rigidBody('end_effector');
setFixedTransform(endEffector.Joint, trvec2tform([-0.25, -0.5, 0])); 
addBody(robot, endEffector, 'body3');

% Joint limits (in degrees)
jointLimitsDeg = [-180 180;   % Joint 1: Base rotation
                  -20 20;   % Joint 2: Up/down movement
                  -45 45];  % Joint 3: End-effector control

% Show the robot structure
figure;
show(robot);
title('3-DOF Robotic Arm');

% ===== Forward Kinematics =====
disp('===== Forward Kinematics =====');

% Define test joint angles (degrees)
jointAnglesDeg = [40, 60, -50]; % Change this to test different configurations
jointAnglesRad = deg2rad(jointAnglesDeg); % Convert to radians

config = robot.homeConfiguration();  % This will give the initial joint positions

config = jointAnglesRad(:);  % Convert joint angles to a column vector (3x1)

% Ensure the joint angles are within limits
for i = 1:length(jointAnglesDeg)
    jointAnglesDeg(i) = max(min(jointAnglesDeg(i), jointLimitsDeg(i,2)), jointLimitsDeg(i,1));
end

% Get the end-effector position
tform = getTransform(robot, config, 'end_effector', 'base');
endEffectorPos = tform2trvec(tform);  % Extract position from the transformation matrix

disp('Joint Angles (degrees):');
disp(jointAnglesDeg);
disp('End-Effector Position (X, Y, Z):');
disp(endEffectorPos);

%% ===== Inverse Kinematics =====
disp('===== Inverse Kinematics =====');

% Set up inverse kinematics solver
ik = inverseKinematics('RigidBodyTree', robot);
weights = [1 1 1 1 1 1]; 
initialGuess = homeConfiguration(robot);

% Define the desired end-effector position
targetPos = [6, 7, 9]; % Change this to test different targets
targetPose = trvec2tform(targetPos);

% Solve for joint angles
[configSol, solInfo] = ik('end_effector', targetPose, weights, initialGuess);

jointAnglesIK = rad2deg(configSol);  % Convert the joint angles from radians to degrees


% Ensure the joint angles from inverse kinematics are within limits
for i = 1:length(jointAnglesIK)
    jointAnglesIK(i) = max(min(jointAnglesIK(i), jointLimitsDeg(i,2)), jointLimitsDeg(i,1));
end

disp('Target Position (X, Y, Z):');
disp(targetPos);
disp('Inverse Kinematics Joint Angles (degrees):');
disp(jointAnglesIK);

% Show the robot with IK solution
figure;
show(robot, configSol);
title('Inverse Kinematics Solution');
