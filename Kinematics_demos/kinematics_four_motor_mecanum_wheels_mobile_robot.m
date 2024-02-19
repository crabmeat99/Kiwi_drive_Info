%% Four motors mecanum wheels mobile robot
clear all; clc;
rw = 0.09; % wheels radius
l = 0.11; % half of robots length
w_dim = 0.11;  % half of robots width

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
subplot(6,1,1:2); hold on; grid on; box on; daspect([1 1 1]);
subplot(613); hold on; grid on; box on; 
subplot(614); hold on; grid on; box on; 
subplot(615); hold on; grid on; box on; 
subplot(616); hold on; grid on; box on; 
for j=1:length(vx_ref)
    % Inverse Kinematics; Calculate wheels' angular velocities based on 
    % reference linear and angular velocities of the robot 
    w1 = (-l-w_dim)/rw * w(j) + vx_ref(j)/rw - vy_ref(j)/rw;
    w2 = (+l+w_dim)/rw * w(j) + vx_ref(j)/rw + vy_ref(j)/rw;
    w3 = (+l+w_dim)/rw * w(j) + vx_ref(j)/rw - vy_ref(j)/rw;
    w4 = (-l-w_dim)/rw * w(j) + vx_ref(j)/rw + vy_ref(j)/rw;
    n = 1;
    for i=0:dt:T
        % Forward Kinematics and Odometry; Update robot position and 
        % orientation based on angular velocities of wheels 
        w_real = rw/(4*(+l+w_dim)) * (-w1+w2+w3-w4);
        vx_omni = rw/4 * (w1+w2+w3+w4);
        vy_omni = rw/4 * (-w1+w2-w3+w4);
        vx = cos(theta)*vx_omni - sin(theta)*vy_omni;
        vy = sin(theta)*vx_omni + cos(theta)*vy_omni;
        x = x + vx*dt;
        y = y + vy*dt;
        theta = theta + w_real*dt;

        % Save data for display
        X(n) = x; Y(n) = y; Theta(n) = theta;
        W1(n) = w1; W2(n) = w2; W3(n) = w3; W4(n) = w4;
        n = n+1;
    end
    subplot(6,1,1:2); plot(X, Y, '.'); xlabel("X [m]"); ylabel("Y [m]")
    subplot(613); plot((j-1)*T+(1:(n-1))*dt, W1, '.'); xlabel("t[s]"); ylabel("\omega_{1} [rad/s]"); ylim([-6 25])
    subplot(614); plot((j-1)*T+(1:(n-1))*dt, W2, '.'); xlabel("t[s]");  ylabel("\omega_{2} [rad/s]"); ylim([-6 25])
    subplot(615); plot((j-1)*T+(1:(n-1))*dt, W3, '.'); xlabel("t[s]");  ylabel("\omega_{3} [rad/s]"); ylim([-6 25])
    subplot(616); plot((j-1)*T+(1:(n-1))*dt, W4, '.'); xlabel("t[s]");  ylabel("\omega_{4} [rad/s]"); ylim([-6 25])
end