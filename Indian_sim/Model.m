%% Dynamic model of a generalized mobile robot
% (land-based) in inertial frame
clear all; clc; close all;
%% Simulation parameters (Euler's method)
dt = 0.1;  	% step size
ts = 60;	% total simulation time
t = 0:dt:ts;	% span 0,0.1,0.2,...,9.9,10.
%% Physical parameters
m = 30;
Icz = 0.1;
xbc = 0; ybc = 0;
phi1 = 30; phi2 = 120 + phi1; phi3 = 120 + phi2;
th = [phi1, phi2, phi3, phi1];
a = 0.1; % Wheel radius
l = 0.5; % wheel centre to vehicle frame
w = 0.05; % Half thickness of the wheel
%% Initial conditions
q0 = [0;0;0]; % initial conditions
qdot0 = [0;0;0];
q(:,1) = q0;
q_dot(:,1) = qdot0;
%% Trajectory details
% Trjectory boundary conditions (known)
t_0 = 0; t_v1 = ts/3; t_v2 = 2*ts/3; t_f = ts;
xd = [0,2,2+2*cosd(120),0];
yd = [0,0,2*sind(120),0];
td = [t_0,t_v1,t_v2,t_f];
xdot = [0,0,0,0];
ydot = [0,0,0,0];
%% Trajectory with via point's velocity is specified
tcx = Cubic_via_V(xd,xdot,td);
tcy = Cubic_via_V(yd,ydot,td);
%% Trajectory with point's velocity is not specified
% tcx = Cubic_via_nV(xd,xdot,td);
% tcy = Cubic_via_nV(yd,ydot,td);
%% Numerical integration starts here
for i = 1:length(t)
	%% Desired values based on Cubic polynomial
    if t(i)<t_v1
		x(i) = [1,t(i),t(i)^2,t(i)^3]*tcx(1:4);
		xdot(i) = [0,1,2*t(i),3*t(i)^2]*tcx(1:4);
	    xddot(i) = [0,0,2,6*t(i)]*tcx(1:4);
		y(i) = [1,t(i),t(i)^2,t(i)^3]*tcy(1:4);
		ydot(i) = [0,1,2*t(i),3*t(i)^2]*tcy(1:4);
        yddot(i) = [0,0,2,6*t(i)]*tcy(5:8);
	elseif t(i)<t_v2
		x(i) = [1,(t(i)-t_v1),(t(i)-t_v1)^2,(t(i)-t_v1)^3]*tcx(5:8);
		xdot(i) = [0,1,2*(t(i)-t_v1),3*(t(i)-t_v1)^2]*tcx(5:8);
		xddot(i) = [0,0,2,6*(t(i)-t_v1)]*tcx(5:8);
		y(i) = [1,(t(i)-t_v1),(t(i)-t_v1)^2,(t(i)-t_v1)^3]*tcx(5:8);
		ydot(i) = [0,1,2*(t(i)-t_v1),3*(t(i)-t_v1)^2]*tcy(5:8);
		yddot(i) = [0,0,2,6*(t(i)-t_v1)]*tcy(5:8);
	else
		x(i) = [1,(t(i)-t_v2),(t(i)-t_v2)^2,(t(i)-t_v2)^3]*tcx(9:12);
		xdot(i) = [0,1,2*(t(i)-t_v2),3*(t(i)-t_v2)^2]*tcx(9:12);
		xddot(i) = [0,0,2,6*(t(i)-t_v2)]*tcx(9:12);
		y(i) = [1,(t(i)-t_v2),(t(i)-t_v2)^2,(t(i)-t_v2)^3]*tcx(9:12);
		ydot(i) = [0,1,2*(t(i)-t_v2),3*(t(i)-t_v2)^2]*tcy(9:12);
		yddot(i) = [0,0,2,6*(t(i)-t_v2)]*tcy(9:12);
    end
		
	if t(i) >=t_v1
	    psi_desired = wrapToPi(atan2(ydot(i),xdot(i)));
	else
	    psi_desired = atan2(ydot(i),xdot(i));
	end
	q_desired(:,i) = [x(i);
						y(i);
						psi_desired];
	if xdot(i)==0 && ydot(i)==0
		psi_dot_desired = 0;
	else
		psi_dot_desired = 0;%(yddot(i)/xdor(i) - (ydot(i)
	end
	q_desired_dot(:,i) = [xdot(i);
							ydot(i);
							psi_dot_desired];
	q_desired_double_dot(:,i) = [xddot(i);
								yddot(i);
								0];
	%%system dynamic terms
	q(:,1) = q_desired(:,1);
	q_dot(:,1) = q_desired_dot(:,1);
	psi = q(3,i); % orientation of the vehicle
	% Jacobian matrix
	J_q = [cos(psi),-sin(psi),0;
			sin(psi),+cos(psi),0;
			0,0,1;];
	psidot = q_dot(3,i); % heading velocity
	%% Other effects
	n_v_mu = [-m*psidot^2*(xbc*cos(psi) - ybc*sin(psi));
				-m*psidot^2*(ybc*cos(psi) + xbc*sin(psi));
              0;];
	%% Inertia matrix
	D_mu = [m,0, -m*(ybc*cos(psi) + xbc*sin(psi));
			0,m,  m*(xbc*cos(psi) - ybc*sin(psi));
			-m*(ybc*cos(psi) + xbc*sin(psi)),...
			m*(xbc*cos(psi) - ybc*sin(psi)),...
			Icz + m*(xbc^2 + ybc^2)];
	%% Errors
	q_tilda(:,i) = q_desired(:,i) - q(:,i);
	q_tilda_dot(:,i) = q_desired_dot(:,i) - q_dot(:,i);
		%% input vector based on Computed-torque control
	tau_mu(:,i) = D_mu * (q_desired_double_dot(:,i) ...
		+4*q_tilda_dot(:,i)+4*q_tilda(:,i))+n_v_mu;
	%% Wheel inputs
	Gamma = [cosd(phi1),cosd(phi2),cosd(phi3); 
			sind(phi1),sind(phi2),sind(phi3);
			1,1,1]; % Wheel configuration matrix
	% Individual wheel inputs
	F(:,i) = (0.95-exp(-t(i)))*inv(Gamma)*tau_mu(:,i);
	%% Accelerations
	q_double_dot(:,i) = inv(D_mu)*(Gamma*F(:,i)-n_v_mu);
	% Velocities (time update 1)
	q_dot(:,i+1) = q_dot(:,i) ...
		+ dt*(q_double_dot(:,i));
	% Positions (time update 2)
	q(:,i+1) = q(:,i) + dt*(q_dot(:,i))...
		+1/2*dt^2*(q_double_dot(:,i));
end % numerical_integration ends here

%% Animation
% w = 0.4; l = 0.6;
% box_v = [-1/2,1/2,1/2,-1/2,-1/2;]
%		-w/2,-w/2,w/2,-w/2;];
% for i = 1:10:length(t)
%	R_psi = [cos(q(3,i)), -sin(q(3,i));
%			sin(q(3,i)),+cos(q(3,i));];
%	veh_ani = R_psi * box_v;
%	fill(veh_ani(1,:)+x(i),veh_ani(2,:)+y(i),'y');
%	hold on 
%	plot(xd,yd,'rs','Markerfacecolor','r','markersize',????
%	plot(x,y,'k--');
%	plot(q(1,1:i),q(2,1:i),'b-');
%	set(gca,'fontsize',16)
%	xlabel('$x$,[m]','Interpreter','Latex');
%	ylabel('$y$,[m]','Interpreter','Latex');
%	llim = min(min(x),min(y)) - 0.5;
%	ulim = max(max(x),max(y)) + 0.5;
%	axis([llim ulim llim ulim]);
%	axis square
%	grid on 
%	pause(0.1)
%	hold off
% end

x = q(1,:);
y = q(2,:);
for i = 1:5:length(t)
	psi=q(3,i);
	R = [cos(psi),-sin(psi);sin(psi),cos(psi)];
	veh_s = R*([l*cosd(th);l*sind(th)]);
	wheel_g = [-a,a,a,-a,-a;-w,-w,w,w,-w];
	roller_g = [-a/3,a/3,a/3,-a/3,-a/3;-w,-w,w,w,-w];
	R1 = [cosd(phi1+90),-sind(phi1+90);
		  sind(phi1+90), cosd(phi1+90);];
	R2 = [cosd(phi2+90),-sind(phi2+90);
		  sind(phi2+90), cosd(phi2+90);];
	R3 = [cosd(phi3+90),-sind(phi3+90);
		  sind(phi3+90), cosd(phi3+90);];
	w_1 = R*(R1*wheel_g + [l*cosd(phi1);l*sind(phi1)]);
	w_2 = R*(R2*wheel_g + [l*cosd(phi2);l*sind(phi2)]);
	w_3 = R*(R3*wheel_g + [l*cosd(phi3);l*sind(phi3)]);
	r_1 = R*(R1*roller_g + [l*cosd(phi1);l*sind(phi1)]);
	r_2 = R*(R2*roller_g + [l*cosd(phi2);l*sind(phi2)]);
	r_3 = R*(R3*roller_g + [l*cosd(phi3);l*sind(phi3)]);


	fill(veh_s(1,:)+x(i),veh_s(2,:)+y(i),'y')
	hold on 
	plot(xd,yd,'rs','Markerfacecolor','r','markersize',5)
	plot([0,1/4*cos(psi)]+x(i),[0,1/4*sin(psi)]+y(i),'r-o',...)
		'Markersize',3,'Markerfacecolor','r','linewidth',2);
	fill(w_1(1,:)+x(i),w_1(2,:)+y(i),'b');
	fill(r_1(1,:)+x(i),r_1(2,:)+y(i),'g');
	fill(w_2(1,:)+x(i),w_2(2,:)+y(i),'b');
	fill(r_2(1,:)+x(i),r_2(2,:)+y(i),'g');
	fill(w_3(1,:)+x(i),w_3(2,:)+y(i),'b');
	fill(r_3(1,:)+x(i),r_3(2,:)+y(i),'g');
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
		a_lx = a_lx - (ay-ax)/2;
	end
	plot(q_desired(1,:),q_desired(2,:),'k--');
	plot(x(1:i),y(1:i),'m-');
	axis ([-1.2*l+b_lx,1.2*l+a_lx,-1.2*l+b_ly,1.2*l+a_ly])
	axis square
	grid on;
	pause(0.1)
	hold off;
end
%% Results
figure
plot(t,q(1,1:i),'r--',t,q(2,1:i),'b-.',t,q(3,1:i),'g-')
%legend('$x$,[m]','$y$,[m]','$\psi$,[rad]','Interpreter','blank','blank','blank')
set(gca,'fontsize',16)
grid on 
xlabel('$t$,[s]','Interpreter','Latex');
ylabel('$q$,[units]','Interpreter','Latex');

function tc = Cubic_via_nV(xd,xdot,td)
	t1 = td(2) - td(2);
	t2 = td(3) - td(2);
	t3 = td(4) - td(3);
	%% Coefficient matrix, A 
	A =[1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0;
	    1 , t1 , t1^2 , t1^3, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ;
	    0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0;
	    0 , 0 , 0 , 0 , 1 , t2 , t2^2 , t2^3 , 0 , 0 , 0 , 0;
	    0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0;
	    0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , t3, t3^2, t3^3;
	    0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0;
	    0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 2*t3, 3*t3^2;
	    0 , 1 , 2*t1 , 3*t1^2 , 0 , -1 , 0 , 0 , 0 , 0 , 0 , 0;
	    0 , 0 , 2 , 6*t1 , 0 , 0 , -2 , 0 , 0 , 0 , 0 , 0;
	    0 , 0 , 0 , 0 , 0 , 1 , 2*t2 , 3*t2^2 , 0 , -1 , 0 , 0;
	    0 , 0 , 0 , 0 , 0 , 0 , 2 , 6*t2 , 0 , 0 , -2 , 0, ];
	%% known inputs
	b = [xd(1);xd(2);xd(2);xd(3);xd(3);xd(4);xdot(1);xdot(4);xdot(1);xdot(1);xdot(1);xdot(1)];
	%% Trajectory coeeficients
	tc = inv(A)*b;
end

function tc = Cubic_via_V(xd,xdot,td)
    t1 = td(2) - td(1);
    t2 = td(3) - td(2);
    t3 = td(4) - td(3);
    A1 = [1 , 0 , 0 , 0 ;
	      0 , 1 , 0 , 0 ;
	      1 , t1 , t1^2 , t1^3 ;
	      0 , 0 , 2*t1 , 3*t1^2;] ;
    A2 = [1 , 0 , 0 , 0 ;
	      0 , 1 , 0 , 0 ;
	      1 , t2 , t2^2, t2^3 ;
	      0 , 0 , 2*t2 , 3*t2^2;] ;
    A3 =  [1 , 0 , 0 , 0 ;
	       0 , 1 , 0 , 0 ;
	       1 , t3 , t3^2 , t3^3 ;
	       0 , 0 , 2*t3 , 3*t3^2;] ;
    b1 = [xd(1); xdot(1); xd(2); xdot(2)];
    b2 = [xd(2); xdot(2); xd(3); xdot(3)];
    b3 = [xd(3); xdot(3); xd(4); xdot(4)];
    %% Coefficient matrix, A
    A = [A1, zeros(4,8);
         zeros(4,4),A2,zeros(4,4);
         zeros(4,8),A3];
    %% known inputs
    b = [b1;b2;b3];
    %% Trajectory coeeficients
    tc = inv(A)*b;
end