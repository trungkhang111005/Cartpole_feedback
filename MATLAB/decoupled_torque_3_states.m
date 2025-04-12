% Physical parameters
g = 9.81;                    
l = 0.15;                    % Length to center of mass of pole [m]
m_cart = 0.80;               % Cart mass [kg]
r = 0.071 / (2*pi);         % Effective drive radius from rev → m 

% New input is torque [Nm]
B_tau = [0;
         1/(l * m_cart * r);
         1/(m_cart * r)];

% State matrix remains same
A = [0        1     0;
     g/l      0     0;
     0        0     0];

% Output: full state
C = eye(3);
D = zeros(3,1);

% State-space system with torque input
sys_tau = ss(A, B_tau, C, D);

% LQR tuning
Q = diag([100 , 10 , 1]);   % [theta, theta_dot, cart_vel]
R = 5.0;                   % Lower R → stronger control since torque is now physical

% Compute LQR gain
K = lqr(A, B_tau, Q, R)

% Closed-loop dynamics
Acl = A - B_tau * K;
sys_cl = ss(Acl, [], eye(3), []);

% Initial condition: 5 deg tilt
x0 = [deg2rad(5); 0; 0];
t = 0:0.001:5;

% Simulate
[~, t, x] = initial(sys_cl, x0, t);

% Plotting
subplot(3,1,1); plot(t, x(:,1)*180/pi); ylabel('\theta (deg)'); title('Pole Angle');
subplot(3,1,2); plot(t, x(:,2)); ylabel('d\theta/dt');
subplot(3,1,3); plot(t, x(:,3)); ylabel('Cart Velocity (m/s)'); xlabel('Time (s)');
