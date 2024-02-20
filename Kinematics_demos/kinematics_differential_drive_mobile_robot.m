%% Differential drive mobile robot
clear all; clc; 
rw = 0.09; % wheels radius
l = 0.11; %distance between wheels

% Reference values
v = [1, 0.5, 2, 1]; 
w = [0.1, pi/2, 0, -0.2 ]; 

% Initial conditions
x = 0;
y = 0;
theta = 0;

% Simulation parameters
dt = 0.001; 
T = 2;

figure; 
subplot(311); hold on; grid on; box on; daspect([1 1 1]);
subplot(312); hold on; grid on; box on; 
subplot(313); hold on; grid on; box on; 
for j=1:length(v)
    % Inverse Kinematics; Calculate wheels' angular velocities based on 
    % reference linear and angular velocities of the robot 
    wl = v(j)/rw - l*w(j)/(2*rw);
    wr = v(j)/rw + l*w(j)/(2*rw);
    n = 1;
    for i=0:dt:T
        % Forward Kinematics and Odometry; Update robot position and 
        % orientation based on angular velocities of wheels 
        v_robot = rw/2 * (wr+wl);
        w_real = rw/l * (wr-wl);

        vx = v_robot*cos(theta);
        vy = v_robot*sin(theta);
        x = x + vx*dt;
        y = y + vy*dt;
        theta = theta + w_real*dt;

        % Save data for display
        X(n) = x; Y(n) = y; Theta(n) = theta;
        WL(n) = wl; WR(n) = wr;
        n = n+1;
    end
    subplot(311); plot(X, Y, '.'); xlabel("X [m]"); ylabel("Y [m]")
    subplot(312); plot((j-1)*T+(1:(n-1))*dt, WL, '.'); xlabel("t[s]"); ylabel("\omega_{l} [rad/s]"); ylim([0 25])
    subplot(313); plot((j-1)*T+(1:(n-1))*dt, WR, '.'); xlabel("t[s]");  ylabel("\omega_{r} [rad/s]"); ylim([0 25])
end