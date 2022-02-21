clc;
syms M m1 m2 l1 l2 g;
load("param.mat");
% From previous question we got for condition 1, 3 and 4 it is observable
% Condition 1
C1 = [1 0 0 0 0 0];

% Condition 3
C3 = [1 0 0 0 0 0;
      0 0 0 0 1 0];

% Condition 4
C4 = [1 0 0 0 0 0;
      0 0 1 0 0 0;  
      0 0 0 0 1 0];

% LQR Controller
R = 0.00001;
Q = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 100 0 0 0;
     0 0 0 500 0 0;
     0 0 0 0 250 0;
     0 0 0 0 0 2000];

% Initial Conditions 
% Linearised A matrix  
A=[0 1 0 0 0 0; 
    0 0 -(m1*g)/M 0 -(m2*g)/M 0;
    0 0 0 1 0 0;
    0 0 -((M+m1)*g)/(M*l1) 0 -(m2*g)/(M*l1) 0;
    0 0 0 0 0 1;
    0 0 -(m1*g)/(M*l2) 0 -(g*(M+m2))/(M*l2) 0];

% Linearised B matrix
B=[0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)];
C = C1;
D = 0;

% LQR Controller
[K, P, Poles] = lqr(A,B,Q,R);
L1 = place(A',C',Poles');

disp(size(L1))
% therefore, the initial conditions are as follows.
% x = [state ; estimate]
x_inLue = [0;0;pi/12;0;pi/6;0;0;0;0;0;0;0]; 

A_l = [A-B*K B*K;
    zeros(size(A)) (A-L1'*C1)];
B_l = [B; zeros(size(B))];
C_l = [C, zeros(size(C))] ;
D_l = D ; 

% State space representation
sys1 = ss(A_l,B_l,C_l,D_l);

figure
% Checking the response 
initial(sys1,x_inLue)

figure
step(sys1)
grid on



