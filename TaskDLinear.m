clc;
syms M m1 m2 l1 l2 g;
load("param.mat");
% Linearised A matrix  
A=[0 1 0 0 0 0; 
    0 0 -(m1*g)/M 0 -(m2*g)/M 0;
    0 0 0 1 0 0;
    0 0 -((M+m1)*g)/(M*l1) 0 -(m2*g)/(M*l1) 0;
    0 0 0 0 0 1;
    0 0 -(m1*g)/(M*l2) 0 -(g*(M+m2))/(M*l2) 0];

% Linearised B matrix
B=[0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)];

% Here n = 6 we will need matrix as 
% Writing the controllability matrix as follows
% cont = [B A*B (A^2)*B (A^3)*B (A^4)*B (A^5)*B];
cont =ctrb(A, B);
cont_subs = subs(cont,M, m1);

disp("Controllability matrix");
disp(cont_subs);
disp("Determinant of matrix = ");
detr = det(cont_subs);
disp(detr);

% Check if the system is controllable if rank of controllability matrix = 6
r = rank(cont_subs);
if(r == 6)
   disp("The system is controllable");
else
    disp("The system is not controllable");
end

% LQR Controller
R = 0.00001;
Q = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 100 0 0 0;
     0 0 0 500 0 0;
     0 0 0 0 250 0;
     0 0 0 0 0 2000];

% therefore, the initial conditions are as follows.
% x = [x x_dot, theta1, theta1_dot, theta2, theta2_dot]
x_initial = [0;0;pi/6;0;pi/3;0]; 
ref = [0; 0; 0; 0; 0; 0];      % reference position
C = eye(6);
D = 0; 

% State space representation
sys1 = ss(A,B,C,D);
figure
% Checking the response
initial(sys1,x_initial)
grid on

% Applying LQR controller to the system
[K, P, Poles] = lqr(A,B,Q,R);

% Feedback matrix
disp('Feedback matrix ')
disp(K)

%Positive definite matrix P 
disp('Positive definite matrix for Lyapunov stability')
disp(P)

disp('Poles for stabilty')
disp(Poles)

Ak = A-B*K;

% After applying feedback
sys2 = ss(Ak,B,C,D); 
figure
initial(sys2,x_initial)
grid on

