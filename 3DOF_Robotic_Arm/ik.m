%% 2-D Path Tracing With Inverse Kinematics
%% Introduction
% This example shows how to calculate inverse kinematics for a simple 2D
% manipulator using the <docid:robotics_ref.bvdhj7x-1 InverseKinematics> class.
% The manipulator robot is a simple 2-degree-of-freedom planar 
% manipulator with revolute joints which is created by assembling rigid bodies into 
% a <docid:robotics_ref.bvan8uq-1 rigidBodyTree> object. A circular trajectory is
% created in a 2-D plane and given as points to the inverse kinematics solver. The solver
% calculates the required joint positions to achieve this trajectory.
% Finally, the robot is animated to show the robot configurations that
% achieve the circular trajectory.
%% Construct The Robot
% Create a |rigidBodyTree| object and rigid bodies with their
% associated joints. Specify the geometric properties of each rigid body
% and add it to the robot.

%% 
% Start with a blank rigid body tree model.
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',4);
%%
% Specify arm lengths for the robot arm.
L1 = 1.2;
L2 = 2.4;
L3 = 1.8;
%%
% Add |'link1'| body with |'joint1'| joint.
body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');
%%
% Add |'link2'| body with |'joint2'| joint.
body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');
%%
% Add |'link3'| body with |'joint3'| joint.
body = rigidBody('link3');
joint = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint, trvec2tform([L2,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link2');
%%
% Add |'tool'| end effector with |'fix1'| fixed joint.
body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L3, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link3');

%%
% Show details of the robot to validate the input properties. The robot
% should have two non-fixed joints for the rigid bodies and a fixed body
% for the end-effector.
showdetails(robot)

%% Define The Trajectory
% Define a circle to be traced over the course of 10 seconds. This circle
% is in the _xy_ plane with a radius of 0.15.
t1 = (0:0.2:10)'; % Time
count1 = length(t1);
% center = [0.5 0.2 0];
% radius = 0.2;
% theta = t*(2*pi/t(end));
%points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];
% points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

l1 = 1.0;
l2 = 2.0;
radius = 0.2;
count = 0;

start_pt1 = [0.5 0.2 0];
end_pt1 = [0.5+l1 0.2 1.0];
points1 = start_pt+t1*(end_pt1 - start_pt1)/count1;
count = count + count1;

t2 = (0:0.2:20)';
count2 = length(t2);

start_pt2 = end_pt1;
end_pt2 = [1 0.2 1.0];
points2 = start_pt2 + t1*(end_pt2 - start_pt2)/count1;
count = count + count1;

start_pt3 = end_pt2;
end_pt3 = [1 -1.8 0];
% print(count2);
points3 = start_pt3 + t2*(end_pt3 - start_pt3)/count2;
count = count + count2;

theta = t2*(pi/t2(end));
center = [0.8 -1.8, 0];
points4 = center + radius*[cos(theta) sin(-theta) zeros(size(theta))]; 
count = count + count2;

points = [
    points1; 
    points2;
    points3;
    points4
    ];

%% Inverse Kinematics Solution
% Use an |inverseKinematics| object to find a solution of robotic 
% configurations that achieve the given end-effector positions along the 
% trajectory. 

%%
% Pre-allocate configuration solutions as a matrix |qs|.
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);
%%
% Create the inverse kinematics solver. Because the _xy_ Cartesian points are the
% only important factors of the end-effector pose for this workflow, 
% specify a non-zero weight for the fourth and fifth elements of the 
% |weight| vector. All other elements are set to zero.
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';

%%
% Loop through the trajectory of points to trace the circle. Call the |ik|
% object for each point to generate the joint configuration that achieves
% the end-effector position. Store the configurations to use later.

qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

%% Animate The Solution
% Plot the robot for each frame of the solution using that specific robot 
% configuration. Also, plot the desired trajectory.

%%
% Show the robot in the first configuration of the trajectory. Adjust the 
% plot to show the 2-D plane that circle is drawn on. Plot the desired 
% trajectory.
figure
show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
axis([-0.1 0.7 -0.3 0.5])

%%
% Set up a <docid:robotics_ref.mw_9b7bd9b2-cebc-4848-a38a-2eb93d51da03 Rate> object to display the robot 
% trajectory at a fixed rate of 15 frames per second. Show the robot in
% each configuration from the inverse kinematic solver. Watch as the arm
% traces the circular trajectory shown.
framesPerSecond = 15;
r = rateControl(framesPerSecond);
% for i = 1:count
%     show(robot,qs(i,:)','PreservePlot',false);
%     drawnow
%     waitfor(r);
% end

site_name = "http://192.168.4.1/test";
qs = qs*180/pi;
qs = int64(qs);
for i = 1:count
    str = string(qs(i, 1));
    str = str + " ";
    str = str + string(qs(i, 2));
    str = str + " ";
    str = str + string(qs(i, 3));
    fprintf(str)
    
    for j = 1:3
        response = webwrite(site_name, qs(i, j));
        pause(0.5);
    end
end

%% 
% Copyright 2012 The MathWorks, Inc.