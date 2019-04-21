function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0
z=2.5;
r = 5;
T= 11;  
if t >= T
    pos = [5;0;2.5];
    vel = [0;0;0];
    acc = [0;0;0];
else
    % quintic polynomial
    t0      = 0;
    tf      = T;
    %%%%%%%%B=H*A%%%%%%%%
    H  =[1   t0  t0^2    t0^3    t0^4    t0^5;
        0    1   2*t0    3*t0^2  4*t0^3  5*t0^4;
        0    0   2       6*t0    12*t0^2 20*t0^3;
        1    tf  tf^2    tf^3    tf^4    tf^5;
        0    1   2*tf    3*tf^2  4*tf^3  5*tf^4;
        0    0   2       6*tf    12*tf^2 20*tf^3];
    b  =[0    0;
        0    0;
        0    0;
        2*pi z;
        0    0;
        0    0];
    %%%%%%%%%%%%% a is 2*1 matrix%%%%%%%%%%%%%%%%%%%
    a = inv(H)*(b);
    q     = a(1,:) + a(2,:)*t + a(3,:)*t^2 + a(4,:)*t^3 + a(5,:)*t^4 + a(6,:)*t^5;
    qd    = a(2,:) + 2*a(3,:)*t + 3*a(4,:)*t^2 + 4*a(5,:)*t^3 + 5*a(6,:)*t^4;
    qdd   = 2*a(3,:) + 6*a(4,:)*t + 12*a(5,:)*t^2 + 20*a(6,:)*t^3
    w    = q(1,1);
    wd   = qd(1,1);
    wdd  = qdd(1,1);
    z       = q(1,2);
    zd      = qd(1,2);
    zdd     = qdd(1,2);
    % position
    x=cos(w)*r;
    y=sin(w)*r;
    pos     = [cos(w)*r; sin(w)*r; z];
    % velocity
    vel     = [-y*wd; x*wd; zd];
    % acceleration
    acc     = [-x*wd^2 - y*wdd; -y*wd^2 + x*wdd; zdd];
end

% yaw and yawdot
yaw = 0;
yawdot = 0;

% output desired state
desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
