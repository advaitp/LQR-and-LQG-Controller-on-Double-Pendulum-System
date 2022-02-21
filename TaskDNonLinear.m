% Clear the output
clc;

% State variables
x0 = [0;0;pi/6;0;pi/3;0];

% timespan to run the non linear system in iteration
time_span = 0:0.01:400;

% initial external force
force = 0; 

% Running the state variables and updating them for 400 steps
[t1,x1,F1] = ode45(@cranepend,time_span,x0,force);
x = x1(:, 1);
x_dot = x1(:, 2);
theta1 = x1(:, 3);
theta1dot = x1(:, 4);
theta2 = x1(:, 5);
theta2dot = x1(:, 6);

% Plotting Graphs 
figure(1)
t = tiledlayout(3,3);
nexttile
plot(t1,x);
nexttile
plot(t1,x_dot);
nexttile
plot(t1,theta1);
nexttile
plot(t1,theta1dot);
nexttile
plot(t1,theta2);
nexttile
plot(t1,theta2dot);
nexttile
plot(t1,theta2dot);
grid on

% nexttile([2,2]);
figure(2)
ref = [0; 0; 0; 0; 0; 0];      % reference position
% Plot
tspan = 0:.01:200;
[p,x,F2] = ode45(@cranepend,tspan,x0,force);
u=@(x)-K*(x - ref); % control law
for k=1:50:length(p)
    livepend(x(k,:));
end

function dxdt = cranepend(t,x,force)
M=1000;
m1=100;
m2=100;
l1=20;
l2=10;
g=10;

A=[0 1 0 0 0 0; 
    0 0 -(m1*g)/M 0 -(m2*g)/M 0;
    0 0 0 1 0 0;
    0 0 -((M+m1)*g)/(M*l1) 0 -(m2*g)/(M*l1) 0;
    0 0 0 0 0 1;
    0 0 -(m1*g)/(M*l2) 0 -(g*(M+m2))/(M*l2) 0];

B=[0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)];

% LQR Parameters
R = 0.00001;
Q = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 100 0 0 0;
     0 0 0 500 0 0;
     0 0 0 0 250 0;
     0 0 0 0 0 2000];
[K, P, Poles] = lqr(A,B,Q,R);

F =- K*x ;
dxdt=zeros(6,1);
% y =[x x_dot theta1 theta1dot theta2 theta2dot]
% y_dot = [x_dot x_dot_dot theta1dot theta1dot_dot theta2dot theta2dot_dot]
dxdt(1) = x(2); 
dxdt(2)=(F-(g/2)*(m1*sin(2*x(3))+m2*sin(2*x(5)))-(m1*l1*(x(4)^2)*sin(x(3)))-(m2*l2*(x(6)^2)*sin(x(5))))/(M+m1*((sin(x(3)))^2)+m2*((sin(x(5)))^2));
dxdt(3)= x(4); 
dxdt(4)= (dxdt(2)*cos(x(3))-g*(sin(x(3))))/l1; 
dxdt(5)= x(6); 
dxdt(6)= (dxdt(2)*cos(x(5))-g*(sin(x(5))))/l2; 
end
