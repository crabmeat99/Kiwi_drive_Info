%% Car-like mobile robot
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
    % Inverse Kinematics; Calculate rear wheels angular velocities and 
    % steering orientation based on reference linear and angular 
    % velocities of the robot 
    wrv = v(j)/rw;
    psi = atan(l*w(j)/v(j));
    n = 1;
    for i=0:dt:T
        % Forward Kinematics and Odometry; Update robot position and 
        % orientation based on and angular velocity of rear wheels and 
        % steering orientation 
        vx = rw * cos(theta)* wrv;
        vy = rw * sin(theta)* wrv;
        w_real = rw/l * tan(psi) * wrv;

        x = x + vx*dt;
        y = y + vy*dt;
        theta = theta + w_real*dt;

        % Save data for display
        X(n) = x; Y(n) = y; Theta(n) = theta;
        WRV(n) = wrv; PSI(n) = psi;
        n = n+1;
    end
    subplot(311); plot(X, Y, '.'); xlabel("X [m]"); ylabel("Y [m]")
    subplot(312); plot((j-1)*T+(1:(n-1))*dt, WRV, '.'); xlabel("t[s]"); ylabel("\omega_{rv} [rad/s]"); ylim([0 25])
    subplot(313); plot((j-1)*T+(1:(n-1))*dt, PSI, '.'); xlabel("t[s]");  ylabel("\psi [rad]")
end