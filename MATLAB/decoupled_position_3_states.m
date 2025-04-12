% 4-State Cart-Pole System with Torque Control
% State: [theta; theta_dot; cart_pos; cart_vel]

% Physical Parameters
g = 9.81;
l = 0.15;
m = 0.8;
r = 0.071 / (2*pi);   % drive radius

% State Matrix A
A = [0      1        0          0;
     g/l    0        0     1/(l*r*m);
     0      0        0          1;
     0      0        0      -1/(r*m)];

% Input Matrix B (Torque Input)
B = [0;
     1/(l*r*m);
     0;
     1/(r*m)];

% Output Matrix
C = eye(4);
D = zeros(4,1);

% Define LQR cost matrices
Q = diag([200, 20, 50, 10]);  % Higher penalty on angle and position
R = 2.0;                      % Control effort (Nm^2)

% Compute LQR gain
K = lqr(A, B, Q, R);

% Closed-loop dynamics
Acl = A - B*K;
sys_cl = ss(Acl, [], eye(4), []);

% Simulation setup
x0 = [deg2rad(8); 0; 0.1; 0];  % Initial tilt + cart displacement
t = 0:0.001:5;
[~, t, x] = initial(sys_cl, x0, t);

% Plot results
subplot(4,1,1); plot(t, x(:,1)*180/pi); ylabel('\theta (deg)'); title('Pole Angle');
subplot(4,1,2); plot(t, x(:,2)); ylabel('d\theta/dt');
subplot(4,1,3); plot(t, x(:,3)); ylabel('Cart Pos (m)');
subplot(4,1,4); plot(t, x(:,4)); ylabel('Cart Vel (m/s)'); xlabel('Time (s)');
