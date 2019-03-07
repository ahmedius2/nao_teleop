% Gebze Technical University
% Control and Robotics Applications Lab

% Before running this script, you must run NaoArmsSL.m first

% just some arm angles to start with
lArmAngles = [1.38503 -0.0193124 -1.54517 -1.37153  0.0280995];
rArmAngles = [1.38503  0.0193124  1.54517  1.37153 -0.0280995];

t = tcpip('0.0.0.0', 19345, 'NetworkRole', 'server', 'Timeout', 6000);
t.ByteOrder = 'littleEndian';
fopen(t);
t
%manipThreshold = fread(t, 1, 'float32');
manipThreshold = 30;
manipulability = 0;

% xyz center = haptic x y z
% these will be initial values, Zenom will send the difference that will be
% added to this coordinate
xc = 0.15; % x center
yc = 0;    % y center
zc = 0.02; % z center
alpha = 0; %haptic pitch
r = 0.05; % radius

LARM= 1;  RARM= 2; BOTH_ARMS= 3;
mode= 0;

tic;
while strcmp(t.Status, 'open')
    zenomData  = fread(t, 6, 'float32')';
    mode = zenomData(6);
    if mode == BOTH_ARMS
        x = xc + zenomData(1); y     = yc + zenomData(2); 
        z = zc + zenomData(3); alpha = zenomData(4); 
        r = zenomData(5);
        % target position of hands
        targetLArmPos = SE3([x , y+(r*cos(alpha)) , z-(r*sin(alpha))]) *...
            SE3.Rx(-alpha-(pi/2));
        targetRArmPos = SE3([x , y-(r*cos(alpha)) , z+(r*sin(alpha))]) *...
            SE3.Rx(-alpha+(pi/2));

        lArmAngles = NaoLArmSL.ikcon(targetLArmPos, lArmAngles); %mbwoo
        rArmAngles = NaoRArmSL.ikcon(targetRArmPos, rArmAngles); %mbwoo
    elseif mode == LARM
        lArmAngles = zenomData(1:5);
    elseif mode == RARM
        rArmAngles = zenomData(1:5);
    end
    
    % [x y z] is center point
    if mode == LARM || mode == BOTH_ARMS
        jl = NaoLArmSL.jacob0(lArmAngles);
        eigs = sqrt(eig(jl * (jl.')));
        manipLArm = max(eigs(2:6)) /  min(eigs(2:6));
        manipulability = manipLArm;
    end
    % If we are going to give feedback anyway, we dont need to compute
    % RArm for BOTH_ARMS mode
    if mode == RARM || (mode == BOTH_ARMS && (manipThreshold >= manipLArm))
        jr = NaoRArmSL.jacob0(rArmAngles);
        eigs = sqrt(eig(jr * (jr.')));
        manipulability = max(eigs(2:6)) /  min(eigs(2:6));
    end
    
    if mode == BOTH_ARMS
        fwrite(t, [lArmAngles 0 rArmAngles 0 manipulability], 'float32');
    else
        fwrite(t, manipulability, 'float32'); %mbwoo
    end
    toc;
    % drawing slows the process, sadly
%     NaoLArmSL.plot(lArmAngles);
%     hold on;
%     NaoRArmSL.plot(rArmAngles);
%     hold on;
%     NaoLArmSL.vellipse(lArmAngles);
%     hold on;
%     NaoRArmSL.vellipse(rArmAngles);

end

fclose(t);
