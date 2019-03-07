% Gebze Technical University
% Control and Robotics Applications Lab
% NAO V4 H25 Arm Kinematics with MATLAB Robotics Toolbox Version 10
addpath('~/Documents/MATLAB/rvctools/');
startup_rvc

UpperArmLength = 0.105;
ElbowOffsetY = 0.015;
ShoulderOffsetY = 0.098;
ShoulderOffsetZ = 0.100;
HandOffsetX = 0.05775;
HandOffsetZ = 0.01231;
LowerArmLength = 0.05595;
L(1,5) = Link();
% ShoulderPitch
L(1) = Revolute('d', 0, 'a', 0, 'alpha', pi/2.0, 'offset',pi/2.0, 'qlim', [-2.0857 2.0857]); 
% ShoulderRoll
L(2) = Revolute('d', 0, 'a',ElbowOffsetY , 'alpha',pi/2.0, 'offset',pi/2.0,'qlim', [-0.3142 1.3265]); 
% ElbowYaw
L(3) = Revolute('d', UpperArmLength, 'a',0, 'alpha', pi/2.0, 'offset', pi, 'qlim', [-2.0857 2.0857]); 
% ElbowRoll
L(4) = Revolute('d', 0, 'a', 0, 'alpha', pi/2.0, 'offset', pi,'qlim', [-1.5446 -0.0349]); 
% WristYaw
L(5) = Revolute('d', LowerArmLength+HandOffsetX, 'a',HandOffsetZ, 'alpha', 0,'offset',-pi/2.0,'qlim',[-1.8238 1.8238]);
% end
NaoLArmSL = SerialLink(L, 'name','NaoLArm',...
    'base', transl(0,ShoulderOffsetY,ShoulderOffsetZ)*trotz(-pi/2.0)*troty(-pi/2.0),...
    'tool', troty(-pi/2.0));

% ShoulderRoll, change a for RArm
L(2) = Revolute('d', 0, 'a', -ElbowOffsetY , 'alpha',pi/2.0, 'offset',pi/2.0, 'qlim', [-1.3265 0.3142]); 
L(4) = Revolute('d', 0, 'a', 0, 'alpha', pi/2.0, 'offset', pi, 'qlim', [0.0349 1.5446]); 

NaoRArmSL = SerialLink(L, 'name','NaoRArm',...
    'base', transl(0,-ShoulderOffsetY,ShoulderOffsetZ)*trotz(-pi/2.0)*troty(-pi/2.0), ...
    'tool', troty(-pi/2.0));
