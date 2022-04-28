% Mechatronics System Project - Group 18 
% Clearing of command window and workplace
clear;
close all;
clc;
%%
% Providing position coordinates of different work-stations
A = [0,0,70];       % A = [x,y,z] where: x,y,z are cartesian coordinates
B = [-0,-50,0];     % B = [x,y,z] where: x,y,z are cartesian coordinates
C = [-50,-0,0];     % C = [x,y,z] where: x,y,z are cartesian coordinates
%%
% Robot Operations
homing() % homing of arm, base and gripper
Picking(C) % picking at given position as per Task 3
Placing(A) % placing at given position as per Task 3
Picking(A) % picking at given position as per Task 4
Placing(B) % placing at given position as per Task 4
Picking(B) % picking at given position as per Task 5
Placing(C) % placing at given position as per Task 5

% Variables Assignment
function [motorA,sensorB,motorB,sensorC,motorC,armrotation,baserotation,rotationC,rotationB,rotationA] = robot_conf(Position)
% connecting robot
myrobo = legoev3('usb'); % connection to robot made through usb 
beep (myrobo);           % beep sound ensures connection to robot

%% 
% Inverse Kinematic Model (Equations)
% Length of links of EV3 manipulator
l1 = 120;
l2 = 95;
l3 = 185;
l4 = 110;
% Calculation of joint variables
theta1 = ceil(atan2d(Position(2),Position(1))); % base rotation angle 
theta2 = ceil(45 + asind((l4+(Position(3)-(l1+(l2*sind(45)))))/l3)); % arm rotation angle
%%
% Robot Configuration
motorA = motor(myrobo, 'A'); % gripper motor
motorB = motor(myrobo, 'B'); % arm motor
motorC = motor(myrobo, 'C'); % base motor
%% bias for base rotation
 if theta1==-90
 theta1 = theta1-5
 end
 if theta1 == -180
 theta1 = theta1-10
 end
%%
baserotation = theta1*3;              % required rotation of base motor
% 85 is the range of arm rotation in degree
armrotation =  (85-theta2)*5;         % required rotation of arm motor
sensorC = touchSensor(myrobo,1);      % limit sensor for base motor 
pressedC = readTouch(sensorC);        % sensor value for base motor 
sensorB = touchSensor(myrobo,3);      % limit sensor for arm motor 
pressedB = readTouch(sensorB);        % sensor value for arm motor 
rotationA = readRotation(motorA);     % encoder value for gripper motor
rotationB = readRotation(motorB);     % encoder value for arm motor
rotationC = readRotation(motorC);     % encoder value for base motor

end
% homing of arm, base and gripper 
function [] = homing()
Position = [0,0,0]
[motorA,sensorB,motorB,sensorC,motorC] = robot_conf(Position)
homingB(sensorB,motorB); % homing of arm motor
homingC(sensorC,motorC); % homing of base motor
homingA(motorA);         % homing of gripper motor
end

% Picking contains these operations: base rotation, arm rotation ,opening and closing of gripper
% input argument : position of work station to pick the object
function [] = Picking(Position)
[motorA,sensorB,motorB,sensorC,motorC,armrotation,baserotation,rotationC,rotationB,rotationA] = robot_conf(Position)
% rotation of base motor to reach desired position
base_rotation(motorC,rotationC,baserotation(1))
motorA.Speed= 0; 
% griper opening
griper_open(motorA,rotationA)
% rotation of arm motor to reach desired position
arm_rotation(motorB,rotationB,armrotation(1))
% griper closing
griper_close(motorA,rotationA)
pause(1) % pause of one second for propper holding of object 
% rotation of arm motor to reach extreme upward position (initial position)
homingB(sensorB,motorB);
end

% Placing contains these operations: base rotation, arm rotation ,opening and closing of gripper
% input argument : position of work station to place the object
function [] = Placing(Position)
[motorA,sensorB,motorB,sensorC,motorC,armrotation,baserotation,rotationC,rotationB,rotationA] = robot_conf(Position)
% rotation of base motor to reach desired position
base_rotation(motorC,rotationC,baserotation(1))
% rotation of arm motor to reach desired position
arm_rotation(motorB,rotationB,armrotation(1))
% griper opening
griper_open(motorA,rotationA)
pause(1) % pause of one second for propper placing of object 
% rotation of arm motor to reach extreme upward position (initial position)
homingB(sensorB,motorB)
% homing is done to ensure opening and closing of gripper for consecutive operation 
homingA(motorA)
end
% homing of arm motor
function [] = homingB(sensorB,motorB)
% reading sensor value for arm motor
pressedB = readTouch(sensorB);
while ~pressedB                     % movement of arm until arm sensor is not pressed
    motorB.Speed = -40;             % arm motor speed
    start(motorB);                  % start arm motor
    pressedB = readTouch(sensorB);  % reading sensor value for arm motor
    disp("homing motor B");
end
motorB.Speed = 0;                   % stop arm motor
resetRotation(motorB);              % reset encoder value to zero
rotationB = readRotation(motorB);   % read encoder value for arm motor
end
% homing of base
function [] = homingC(sensorC,motorC)
% reading sensor value for base motor
pressedC = readTouch(sensorC);
while ~pressedC                     % movement of arm until base sensor is not pressed
    motorC.Speed = 40;              % base motor speed
    start(motorC);                  % start base motor
    pressedC = readTouch(sensorC);  % reading sensor value for base motor
    disp("homing motor C");
end
motorC.Speed = 0;                   % stop base motor
resetRotation(motorC);              % reset encoder value to zero
rotationC = readRotation(motorC),   % read encoder value for base motor
end
% homing of gripper
function [] = homingA(motorA)
motorA.Speed = 20;                  % gripper motor speed
start(motorA);                      % start gripper motor
pause(1);                           % pause of one second for proper homing of gripper
motorA.Speed=0;                     % stop gripper motor
rotationA = readRotation(motorA);   % read encoder value for gripper motor
disp("homing motor A");
end
% griper closing
function [] = griper_close(motorA,rotationA)
rotationA = readRotation(motorA);   % read encoder value for gripper motor
resetRotation(motorA);              % reset encoder value of gripper motor
rotationA = 0;                      % set encoder value to zero
while (rotationA < 80)     % moving motor until half of clockwise rotation
    motorA.Speed = 40;              % gripper motor speed in clockwise direction
    start(motorA);                  % start gripper motor
    rotationA = readRotation(motorA); % read encoder value for gripper motor
    disp("Griper_close");
end
motorA.Speed = 0;                   % stop gripper motor

end
% griper opening
function [] = griper_open(motorA,rotationA)
motorA.Speed = 0;                 % stop gripper motor
rotationA = readRotation(motorA); % read encoder value for gripper motor
resetRotation(motorA);            % reset encoder value of gripper motor
rotationA=0;                      % set encoder value to zero
while (rotationA > -80)  % moving motor until half of anti-clockwise rotation
    motorA.Speed = -40;           % gripper motor speed in anti-clockwise direction
    start(motorA);                % start gripper motor
    rotationA = readRotation(motorA); % read encoder value for gripper motor
    disp("Griper_open");
end
motorA.Speed = 0;                 % stop gripper motor
end
% positioning arm
function [] = arm_rotation(motorB,rotationB,armrotation)
% position B
resetRotation(motorB);
rotationB = 0;
% Using PID controller for proper and smooth operation
% Values of parameters of PID controller
Kp = 0.05;  % proportionl gain
Ki = 0.06;  % integral gain
Kd = 0.04;  % differential gain
T = 0.15;   % time constant
ceb = armrotation - rotationB;  % error (desired_position - actual_position)   
e0b = ceb;  % total error
peb = 0;    % previous error
x = [3,4,2,1,0,-1,-2,-3,-4]; % error tolerance range
while  ~ismember(ceb,x)     % move arm until the error not in desired range
    ceb = armrotation - rotationB; % current error
    % discrete PID algorithm
    u1b = Kp*ceb + Ki*T*e0b + ((Kd/T)*(ceb-peb));
    motorB.Speed = u1b/4; % output is given to speed of arm motor
    start(motorB);
    e0b = ceb + e0b;
    peb = ceb;  
    % read encoder value for arm motor
    rotationB = readRotation(motorB)
    disp("rotationB");
end
motorB.Speed = 0; % stop arm motor
end
% positioning base
function [] = base_rotation(motorC,rotationC,baserotation)
rotationC = readRotation(motorC); % read encoder value for base motor
% Using PID controller for proper and smooth operation
% values of parameters of PID controller
Kp = 0.07;  % proportionl gain
Ki = 0.06;  % integral gain
Kd = 0.04;  % differential gain
T = 0.09;   % time constant
ce = baserotation - rotationC; % error (desired_position - actual_position) 
e0 = ce;    % total error
pe = 0;     % previous error
x = [3,4,2,1,0,-1,-2,-3,-4,5,-5];  % error tolerance range
while ~ismember(ce,x)         % move base until the error not in desired range
    ce = baserotation - rotationC;  % current error
    % discrete PID algorithm
    u1 = Kp*ce + Ki*T*e0 + ((Kd/T)*(ce-pe));
    motorC.Speed = u1/2.5  % output is given to speed of arm motor
    start(motorC);
    e0 = ce + e0;
    pe = ce;
    % read encoder value for base motor
    rotationC = readRotation(motorC);
    disp("rotationC ");
end
motorC.Speed = 0; % stop base motor
end
