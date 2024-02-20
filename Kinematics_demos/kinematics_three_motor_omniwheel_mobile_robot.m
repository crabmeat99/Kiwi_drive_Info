%% Three motors omniwheel mobile robot 
clear all; clc; 
rw = 0.09; % wheels radius
d = 0.11; %distance between wheels and center of the robot

% Reference values
vx_ref = [1, 0.5, 0, 1];
vy_ref = [0, 0, 0.5, 0]; 
w = [0.1, pi/2, 0, -0.2 ]; 

% Initial conditions
x = 0;
y = 0;
theta = 0;

% Simulation parameters
dt = 0.001; 
T = 2;

figure; 
subplot(5,1,1:2); hold on; grid on; box on; daspect([1 1 1]);
subplot(513); hold on; grid on; box on; 
subplot(514); hold on; grid on; box on; 
subplot(515); hold on; grid on; box on; 
for j=1:length(vx_ref)
    % Inverse Kinematics; Calculate wheels' angular velocities based on 
    % reference linear and angular velocities of the robot 
    w1 = -d/rw * w(j) + vx_ref(j)/rw;
    w2 = -d/rw * w(j) - vx_ref(j)/(2*rw) - sin(pi/3)*vy_ref(j)/rw;
    w3 = -d/rw * w(j) - vx_ref(j)/(2*rw) + sin(pi/3)*vy_ref(j)/rw;
    n = 1;
    for i=0:dt:T
        % Forward Kinematics and Odometry; Update robot position and 
        % orientation based on angular velocities of wheels 
        w_real = rw/(3*d) * (-w1-w2-w3);
        vx_omni = rw/3 * (2*w1-w2-w3);
        vy_omni = rw / (2*sin(pi/3)) * (-w2+w3);
        vx = cos(theta)*vx_omni - sin(theta)*vy_omni;
        vy = sin(theta)*vx_omni + cos(theta)*vy_omni;
        x = x + vx*dt;
        y = y + vy*dt;
        theta = theta + w_real*dt;

        % Save data for display
        X(n) = x; Y(n) = y; Theta(n) = theta;
        W1(n) = w1; W2(n) = w2; W3(n) = w3;
        n = n+1;
    end
    subplot(5,1,1:2); plot(X, Y, '.'); xlabel("X [m]"); ylabel("Y [m]")
    subplot(513); plot((j-1)*T+(1:(n-1))*dt, W1, '.'); xlabel("t[s]"); ylabel("\omega_{1} [rad/s]"); ylim([-6 25])
    subplot(514); plot((j-1)*T+(1:(n-1))*dt, W2, '.'); xlabel("t[s]");  ylabel("\omega_{2} [rad/s]"); ylim([-6 25])
    subplot(515); plot((j-1)*T+(1:(n-1))*dt, W3, '.'); xlabel("t[s]");  ylabel("\omega_{3} [rad/s]"); ylim([-6 25])
end