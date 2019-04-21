    function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
%% Inputs:
%
% qd{qn}: state and desired state information for quadrotor #qn (qn
%         will be = 1 since we are only flying a single robot)
%
%  qd{qn}.pos, qd{qn}.vel   position and velocity
%  qd{qn}.euler = [roll;pitch;yaw]
%  qd{qn}.omega     angular velocity in body frame
% 
%  qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des  desired position, velocity, accel
%  qd{qn}.yaw_des, qd{qn}.yawdot_des
%
% t: current time
%    
% qn: quadrotor number, should always be 1
%    
% params: various parameters
%  params.I     moment of inertia
%  params.grav  gravitational constant g (9.8...m/s^2)
%  params.mass  mass of robot
%
%% Outputs:
%
% F: total thrust commanded (sum of forces from all rotors)
% M: total torque commanded
% trpy: thrust, roll, pitch, yaw (attitude you want to command!)
% drpy: time derivative of trpy
%
% Using these current and desired states, you have to compute the desired
% controls u, and from there F and M
%

% =================== Your code goes here ===================
% ...
% ==============================

% Desired roll, pitch and yaw (in rad). In the simulator, those will be *ignored*.
% When you are flying in the lab, they *will* be used (because the platform
% has a built-in attitude controller). Best to fill them in already
% during simulation.

phi_des   = 0;
theta_des = 0;
psi_des   = 0;

%
%
%
 u = zeros(4,1); % control input u, you should fill this in
 Kd = [11,11,11];%%%% D control
 Kp = [34,34,34];%%%%%%%P control
 K_R = [300 0 0;0 270 0;0 0 280];
 K_w = [20 0 0;0 22 0;0 0 22];
%%%%%%%%%%%%%%% position error guarantte error goes to zero%%%%%
%%%% acceleration equ 21%%%%
for i=1:3
qd{qn}.acc(i,:) = qd{qn}.acc_des(i)-Kd(i)*(qd{qn}.vel(i)-qd{qn}.vel_des(i))-Kp(i)*(qd{qn}.pos(i)-qd{qn}.pos_des(i));
end 
%%%%%%% attitude of the quad rotor should be controlled%%%%%
%%%equation 17%%%%
Fdes=params.mass*qd{qn}.acc-[0;0;-params.mass*params.grav];
%%%%%R is the aRb which is equation 4%%%%
R = eulzxy2rotmat(qd{qn}.euler);
%%%%%%%Ru1=Ru1
u(1)=params.mass*params.grav+params.mass*qd{qn}.acc(3,:);
%calculation done for u(1)%%%%
%%%calculation begin for u(2) which is the moment%%%%%
%%%%%equation 25%%%%%
%%%%%we should solve for theta desire and phi desire first%%%%%
%%%%%%%equation 22a 22b%%%%%
b3 = R*[0;0;1];
b3_des = Fdes/norm(Fdes);
a_psi=[cos(qd{qn}.yaw_des);sin(qd{qn}.yaw_des);0];
b2_des = cross(b3_des,a_psi)/norm(cross(b3_des,a_psi));
R_des = [cross(b2_des,b3_des),b2_des,b3_des];
qd{qn}.euler_des = rotmat2eulzxy(R_des);
qd{qn}.omega_des = 0;
e_R = veemap(1/2*(R_des'*R-R'*R_des));
e_w = qd{qn}.omega-qd{qn}.omega_des;
u(2:4) = params.I*(-K_R*transpose(e_R)-K_w*e_w);
    % Thrust
F    = u(1);       % This should be F = u(1) from the project handout

% Moment
M    = u(2:4);     % note: params.I has the moment of inertia

% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end

%
% ------------------------------------------------------------
%    should you decide to write a geometric controller,
%    the following functions should come in handy
%

function m = eulzxy2rotmat(ang)
    phi   = ang(1);
    theta = ang(2);
    psi   = ang(3);
    
    m = [[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), ...
          cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)];
         [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), ...
          sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)];
         [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]];
end

function eul = rotmat2eulzxy(R)
    if R(3,2) < 1
        if R(3,2) > -1
            thetaX = asin(R(3,2));
            thetaZ = atan2(-R(1,2), R(2,2));
            thetaY = atan2(-R(3,1), R(3,3));
        else % R(3,2) == -1
            thetaX = -pi/2;
            thetaZ = -atan2(R(1,3),R(1,1));
            thetaY = 0;
        end
    else % R(3,2) == +1
        thetaX = pi/2;
        thetaZ = atan2(R(1,3),R(1,1));
        thetaY = 0;
    end
    eul = [thetaX, thetaY, thetaZ];
end

function w = veemap(R)
    w = [-R(2,3), R(1,3), -R(1,2)];
end
