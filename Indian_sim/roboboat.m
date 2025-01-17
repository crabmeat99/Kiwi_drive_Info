clear all;
close all;
clc;

dt = 0.1; %step time
ts = 30;  %total duration of the sim
t = 0:dt:ts; %timespan?

phi1 = 60; phi2 = 120 + phi1; phi3 = 120 + phi2;
th = [phi1,phi2,phi3,phi1]; % The angle of three wheels from the center 

a = 0.1; % Wheel radius 10 cm
l = 0.5; % Wheel centre to vehicle frame
w = 0.025; % Half thickness of the wheel
eta(:,1) = [0;0;0];
rx = 2; ry = 1; wx = 0.1; wy = 0.1;
for i = 1:length(t)
	xd = 0.05*t(i); yd = 0.05*t(i); psid = pi/4; % TRY: change pi/4 to 0
	eta_d(:,i) = [xd;yd;psid];  % TRY: change psid to 0*psid 
	e(:,i) = eta_d(:,i) - eta(:,i);
	J_eta = [cos(eta(3,i)),-sin(eta(3,i)),0;
			sin(eta(3,i)), cos(eta(3,i)),0;
			0,0,1];
	zeta(:,i) = inv(J_eta)*4*e(:,i); 
	%W_inv = 3*1/a*[-sind(phi1),cosd(phi1),l;
		%-sind(phi2),cosd(phi2),l;
		%-sind(phi3),cosd(phi3),l;];
    W_inv = a *[0 , -1, l; cos(pi/6), sin(pi/6), l; -cos(pi/6), sin(pi/6), l];
    omega(:,i) = (0.9-exp(-t(i)))*W_inv*zeta(:,i); %accounts for momentum starts slow 
	W = inv(W_inv) %Back to world 
	eta_dot(:,i) = J_eta*W*omega(:,i); % adds pos to eta for next eta
	eta(:,i+1) = eta(:,i) + dt*eta_dot(:,i);
end

x = eta(1,:);
y = eta(2,:);
for i = 1:5:length(t)
    psi = eta(3,i);
    R = [cos(psi), -sin(psi);sin(psi),cos(psi)];
    veh_s = R*([l*cosd(th);l*sind(th)]);
    wheel_g = [-a,a,a,-a,-a;-w,-w,w,w,-w]; %
    roller_g = [-a/3,a/3,a/3,-a/3,-a/3;-w,-w,w,w,-w];
    R1 = [cosd(phi1+90),-sind(phi1+90);
		    sind(phi1+90), cosd(phi1+90);]; %wheels rotating
    R2 = [cosd(phi2+90),-sind(phi2+90);
		    sind(phi2+90), cosd(phi2+90);];
    R3 = [cosd(phi3+90),-sind(phi3+90);
		    sind(phi3+90) cosd(phi3+90);];
    w_1 = R*(R1*wheel_g + [l*cosd(phi1);l*sind(phi1)]);
    w_2 = R*(R2*wheel_g + [l*cosd(phi2);l*sind(phi2)]);
    w_3 = R*(R3*wheel_g + [l*cosd(phi3);l*sind(phi3)]);
	r_1 = R*(R1*(roller_g-[a-a/2*((sawtooth(omega(1,i)*t(i))+1.2));0.0]) + [l*cosd(phi1);l*sind(phi1)]);
    r_11 = R*(R1*(roller_g+[a/2*((sawtooth(omega(1,i)*t(i))+1));0.0]) + [l*cosd(phi1);l*sind(phi1)]);
    r_2 = R*(R2*(roller_g-[a-a/2*((sawtooth(omega(2,i)*t(i))+1.2));0.0]) + [l*cosd(phi2);l*sind(phi2)]);
    r_22 = R*(R2*(roller_g+[a/2*((sawtooth(omega(2,i)*t(i))+1));0.0]) + [l*cosd(phi2);l*sind(phi2)]);
    r_3 = R*(R3*(roller_g-[a-a/2*((sawtooth(omega(3,i)*t(i))+1.2));0.0]) + [l*cosd(phi3);l*sind(phi3)]);
    r_33 = R*(R3*(roller_g+[a/2*((sawtooth(omega(3,i)*t(i))+1));0.0]) + [l*cosd(phi3);l*sind(phi3)]);

    % r_2 = R*(R2*(roller_g) + (l*cosd(phi2);l*sind(phi2)]);
    % r_3 = R*(R3*(roller_g) + (l*cosd(phi3);l*sind(phi3)]);
    
    fill(veh_s(1,:)+x(i),veh_s(2,:)+y(i),'y')
	hold on 
	plot([0,1/4*cos(psi)]+x(i),[0,1/4*sin(psi)]+y(i),'r-o',...)
		'Markersize',3,'Markerfacecolor','r','linewidth',2);
    fill(w_1(1,:)+x(i),w_1(2,:)+y(i),'b');
	fill(r_1(1,:)+x(i),r_1(2,:)+y(i),'g');
    fill(r_11(1,:)+x(i),r_11(2,:)+y(i),'g');
	fill(w_2(1,:)+x(i),w_2(2,:)+y(i),'b');
	fill(r_2(1,:)+x(i),r_2(2,:)+y(i),'g');
    fill(r_22(1,:)+x(i),r_22(2,:)+y(i),'g');
	fill(w_3(1,:)+x(i),w_3(2,:)+y(i),'b');
	fill(r_3(1,:)+x(i),r_3(2,:)+y(i),'g');
    fill(r_33(1,:)+x(i),r_33(2,:)+y(i),'g');
    a_lx = max(x);
	b_lx = min(x);
	a_ly = max(y);
	b_ly = min(y);
	ax = a_lx-b_lx;
	ay = a_ly-b_ly;
	if ax>ay 
		b_ly = b_ly - (ax-ay)/2;
		a_ly = a_ly + (ax-ay)/2;
	else
		b_lx = b_lx - (ay-ax)/2;
        a_lx = a_lx + (ay-ax)/2;
    end	
    plot(eta_d(1,:),eta_d(2,:),'k--'); % plot actual path
    plot(x(1:i),y(1:i),'m-'); % plot live path over actual path
    axis ([-0.6+b_lx,0.6+a_lx,-0.6+b_ly,0.6+a_ly])
    axis square
    grid on;
    pause(0.1)
    hold off;
end
%% Results
figure
plot(t,eta(1,1:i),'r--',t,eta(2,1:i),'b-.',t,eta(3,1:i),'g-')
legend('$x$,[m]','$y$,[m]','$\psi$,[rad]','Interpreter','Latex');
set(gca,'fontsize',16)
grid on 
xlabel('$t$,[s]','Interpreter','Latex');
ylabel('$eta$,[units]','Interpreter','Latex');  
figure
plot(t,omega)
legend('$w1$,[v]','$w2$,[v]','$w3$,[v]','Interpreter','Latex')
set(gca,'fontsize',16)
grid on 
xlabel('$t$,[s]','Interpreter','Latex');
ylabel('$v$,[units]','Interpreter','Latex');




    