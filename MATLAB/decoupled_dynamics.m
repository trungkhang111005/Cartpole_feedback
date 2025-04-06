% Physical parameters
g = 9.81;      % Gravity [m/s^2]
l = 0.12962;       % Pendulum length to center of mass [m]
tau = 0.05 ;

A = [0 1 0 0; g/l 0 0 1/(l*tau); 0 0 0 1; 0 0 0 -1/tau];
B = [0; -1/(l*tau); 0; 1/tau];
C = eye(4);
D = [0; 0 ; 0; 0];
sys_vel = ss(A,B,C,D);
%% 
sys_vel;
Q = diag([100 , 10 ,10 ,1]);
R = 0.1;
K = lqr(A,B,Q,R);
%% 
Acl = A - B*K;                 % Closed-loop dynamics
sys_cl = ss(Acl, [], eye(4), []);
x0 = [deg2rad(5); 0; 0; 0];    % 5° pole tilt, everything else zero
t = 0:0.001:5;
%%
[y, t, x] = initial(sys_cl, x0, t);
%% 
subplot(4,1,1);
plot(t, x(:,1) * 180/pi); ylabel('\theta (deg)');
title('Pole Angle');

subplot(4,1,2);
plot(t, x(:,2)); ylabel('d\theta/dt');

subplot(4,1,3);
plot(t, x(:,3)); ylabel('Cart Position (m)');

subplot(4,1,4);
plot(t, x(:,4)); ylabel('Cart Velocity (m/s)');
xlabel('Time (s)');
