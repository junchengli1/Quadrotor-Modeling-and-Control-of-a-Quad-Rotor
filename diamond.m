function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0
if t <= 1
    acc = [1/4; sqrt(2); sqrt(2)];
    vel = [t/4; sqrt(2)*t; t*sqrt(2)];
    pos = [t^2/8; sqrt(2)*t^2/2; sqrt(2)*t^2/2];
    
elseif t >1 && t<=2
   t=t-1;
   acc=[-1/4; -sqrt(2); -sqrt(2)];
   vel=[1/4; sqrt(2);sqrt(2)]+acc*t;
   pos=[1^2/8; sqrt(2)*1^2/2; sqrt(2)*1^2/2]+[1/4; sqrt(2);sqrt(2)]*t+1/2*acc*t^2;
elseif t>2 && t <=3
    t=t-2;
 acc = [1/4;-sqrt(2);sqrt(2)];
 vel = acc*t;
 pos=[1/4;sqrt(2);sqrt(2)]+1/2*acc*t^2;
elseif t>3 && t<=4
    t=t-3;
    acc=[-1/4;sqrt(2);-sqrt(2)];
    vel=[1/4;-sqrt(2);sqrt(2)]+acc*t;
pos=[0.375; sqrt(2)/2; 3/2*sqrt(2)]+[1/4;-sqrt(2);sqrt(2)]*t+1/2*acc*t^2;
elseif t>4 && t<=5
    t=t-4;
acc = [1/4;-sqrt(2);-sqrt(2)];
    vel=acc*t;
    pos=[1/2;0;2*sqrt(2)]+1/2*acc*t^2;

elseif t>5 && t<=6
     t=t-5 ;
    acc = [-1/4;sqrt(2);sqrt(2)];
    vel=[1/4;-sqrt(2);-sqrt(2)]+acc*t;
    pos=[1/2;0;2*sqrt(2)]+1/2*[1/4;-sqrt(2);-sqrt(2)]*1^2+[1/4;-sqrt(2);-sqrt(2)]*t+1/2*acc*t^2;
elseif t>6 && t<=7
    t=t-6 ;
    acc = [1/4;sqrt(2);-sqrt(2)];
    vel=acc*t;
    pos=[3/4;-sqrt(2);sqrt(2)]+1/2*acc*t^2;

elseif t>7 && t<=8
    t=t-7;
    acc = [-1/4;-sqrt(2);sqrt(2)];
    vel=[1/4;sqrt(2);-sqrt(2)]+acc*t;
    pos=[0.8750;-sqrt(2)/2;sqrt(2)/2]+[1/4;sqrt(2);-sqrt(2)]*t+1/2*acc*t^2;
 
else
    pos = [1;0;0];
    vel = [0;0;0];
    acc = [0;0;0];
end

yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
