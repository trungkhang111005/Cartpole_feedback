% Physical parameters
g = 9.81;                     
l = 0.12962;                  
tau = 0.01;                  

% Conversion: RPM to m/s
rpm_to_mps = 0.071/ 60;

% 3-State reduced model: x = [theta; theta_dot; cart_velocity]
A = [  0           1               0;
      g/l          0       1/(l*tau);
       0           0         -1/tau ];

% B in m/s
B_mps = [0; 0; 1/tau];

% Scale B for RPM input
B_rpm = B_mps * rpm_to_mps;

% Output: full state
C = eye(3);
D = zeros(3,1);

% State-space system
sys_rpm = ss(A, B_rpm, C, D);
% LQR weighting matrices
Q = diag([100 , 10 , 1]);   % [theta, theta_dot, cart_vel]
R = 0.1;                    % input cost (in RPM)

% LQR gain for RPM input
K = lqr(A, B_rpm, Q, R);
K
%% 

% Closed-loop dynamics
Acl = A - B_rpm * K;
sys_cl = ss(Acl, [], eye(3), []);

% Initial condition: 5° pole angle, rest zero
x0 = [deg2rad(5); 0; 0];   
t = 0:0.001:5;

% Simulate response
[~, t, x] = initial(sys_cl, x0, t);

% Plot results
subplot(3,1,1);
plot(t, x(:,1)*180/pi); ylabel('\theta (deg)');
title('Pole Angle');

subplot(3,1,2);
plot(t, x(:,2)); ylabel('d\theta/dt');

subplot(3,1,3);
plot(t, x(:,3)); ylabel('Cart Velocity (m/s)');
xlabel('Time (s)');
