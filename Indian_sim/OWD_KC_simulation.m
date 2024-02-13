% eta_d : Desired position
% eta : Current position
% e : error in world coordinates
% zeta : force derived from p * error in body coordinates
% omega : wheel speed

clear all;
close all;
clc;

my_pause = true;

P = 4; 

dt = 0.05; %step time
ts = 30; %Total duration of the sim
t = 0:dt:ts; %timespan


phi1 = 60; phi2 = 120 + phi1; phi3 = 120 + phi2;
th = [phi1,phi2,phi3,phi1]; % The angle of three wheels from the center 

a = 0.1; % Wheel radius 10 cm
l = 0.5; % Wheel centre to vehicle frame
w = 0.025; % Half thickness of the wheel
eta(:,1) = [0;0;0];
rx = 2; ry = 1; wx = 0.1; wy = 0.1;
for i = 1:length(t)
    % Circular path
	xd = rx*sin(wx*t(i));
	yd = ry-ry*cos(wy*t(i));
	xd_dot = rx*wx*cos(wx*t(i));
    yd_dot = ry*wy*sin(wy*t(i));
	if xd_dot==0 && yd_dot ==0
		psid = 0;
	else
		psid = wrapTo2Pi(atan2(yd_dot,xd_dot));
	end
     
    % Linear path 0.05 in x and y.  Constant angle
	% xd = 0.05*t(i); yd = 0.05*t(i); psid =pi/4; % TRY: change pi/4 to 0


	eta_d(:,i) = [xd;yd;psid];  % TRY: change psid to 0*psid 
	e(:,i) = eta_d(:,i) - eta(:,i);
	J_eta = [cos(eta(3,i)),-sin(eta(3,i)),0;
			sin(eta(3,i)), cos(eta(3,i)),0;
			0,0,1];
	zeta(:,i) = inv(J_eta)*P*e(:,i);
	%W_inv = 3*1/a*[-sind(phi1),cosd(phi1),l;
		%-sind(phi2),cosd(phi2),l;
		%-sind(phi3),cosd(phi3),l;]; %Wheel Config
	W_inv = a *[0 , -1, l; cos(pi/6), sin(pi/6), l; -cos(pi/6), sin(pi/6), l];
	omega(:,i) = (0.9-exp(-t(i)))*W_inv*zeta(:,i); % Wheel speedsaccounts for momentum starts slow 
	W = inv(W_inv); %Back to world coordinates
	eta_dot(:,i) = J_eta*W*omega(:,i); %adds Pos to eta for next eta
	eta(:,i+1) = eta(:,i) + dt*eta_dot(:,i);
end

figure('Position',[100 100 1000 1000]);

x = eta(1,:);
y = eta(2,:);
for i = 1:5:length(t)
    psi = eta(3,i);
    R = [cos(psi), -sin(psi);sin(psi),cos(psi)];  % Boat rotation matrix
    veh_s = R*([l*cosd(th);l*sind(th)]);
    wheel_g = [-a,a,a,-a,-a;-w,-w,w,w,-w]; % Wheel, centered rectangle 2a = length, 2w = width
    speed1_g = [0,(omega(3,i)*100);0,0];
    speed2_g = [0,(omega(1,i)*100);0,0];
    speed3_g = [0,(omega(2,i)*100);0,0];
    R1 = [cosd(phi1+90),-sind(phi1+90);
		    sind(phi1+90), cosd(phi1+90);]; % wheels rotating
    R2 = [cosd(phi2+90),-sind(phi2+90);
		    sind(phi2+90), cosd(phi2+90);];
    R3 = [cosd(phi3+90),-sind(phi3+90);
		    sind(phi3+90) cosd(phi3+90);];
    w_1 = R*(R1*wheel_g + [l*cosd(phi1);l*sind(phi1)]);
    w_2 = R*(R2*wheel_g + [l*cosd(phi2);l*sind(phi2)]);
    w_3 = R*(R3*wheel_g + [l*cosd(phi3);l*sind(phi3)]);
    s_1 = R*(R1*speed1_g + [l*cosd(phi1);l*sind(phi1)]);
    s_2 = R*(R2*speed2_g + [l*cosd(phi2);l*sind(phi2)]);
    s_3 = R*(R3*speed3_g + [l*cosd(phi3);l*sind(phi3)]);
    
    %%fill(veh_s(1,:)+x(i),veh_s(2,:)+y(i),'y') % trangle body
    my_circle(x(i),y(i),l);
	hold on 

	plot([0,1/4*cos(psi)]+x(i),[0,1/4*sin(psi)]+y(i),'r-o',...)
		'Markersize',3,'Markerfacecolor','r','linewidth',2);
    fill(w_1(1,:)+x(i),w_1(2,:)+y(i),'b');
	fill(w_2(1,:)+x(i),w_2(2,:)+y(i),'b');
	fill(w_3(1,:)+x(i),w_3(2,:)+y(i),'b');
    % plot(s_1(1,:)+x(i),s_1(2,:)+y(i),'r');
    % plot(s_2(1,:)+x(i),s_2(2,:)+y(i),'r');
    % plot(s_3(1,:)+x(i),s_3(2,:)+y(i),'r');
    quiver(s_1(1,1)+x(i),s_1(2,1)+y(i),s_1(1,2)-s_1(1,1),s_1(2,2)-s_1(2,1),'Color','r','LineWidth',2);
    quiver(s_2(1,1)+x(i),s_2(2,1)+y(i),s_2(1,2)-s_2(1,1),s_2(2,2)-s_2(2,1),'Color','r','LineWidth',2);
    quiver(s_3(1,1)+x(i),s_3(2,1)+y(i),s_3(1,2)-s_3(1,1),s_3(2,2)-s_3(2,1),'Color','r','LineWidth',2);
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
    axis_l = 1.0;           
    axis ([-axis_l+b_lx,axis_l+a_lx,-axis_l+b_ly,axis_l+a_ly])
    axis square
    grid on;
    if my_pause == true;
        pause;
        my_pause = false;
    else 
        pause(0.05);
    end
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

function h = my_circle(x,y,r)
%hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
%%h = plot(xunit, yunit,'g-');
fill(xunit,yunit,'g-');
%hold off
end

    
