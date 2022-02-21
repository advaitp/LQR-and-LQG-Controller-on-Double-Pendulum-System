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
D = 0;

% LQR Controller
[K, P, Poles] = lqr(A,B,Q,R);

% Luenberger Observer
L1 = place(A',C1',Poles');
L3 = place(A',C3',Poles');
L4 = place(A',C4',Poles');

disp(size(L1))
% therefore, the initial conditions are as follows.
% x = [state ; estimate]
x_inLue = [0;0;pi/12;0;pi/6;0;0;0;0;0;0;0]; 

% A matrix 
A_l1 = [A-B*K B*K;
    zeros(size(A)) (A-L1'*C1)];
A_l3 = [A-B*K B*K;
    zeros(size(A)) (A-L3'*C3)];
A_l4 = [A-B*K B*K;
    zeros(size(A)) (A-L4'*C4)];

% B matrix
B_l1 = [B; zeros(size(B))];
B_l3 = [B; zeros(size(B))];
B_l4 = [B; zeros(size(B))];

% C matrix
C_l1 = [C1, zeros(size(C1))] ;
C_l3 = [C3, zeros(size(C3))] ;
C_l4 = [C4, zeros(size(C4))] ;

% D matrix
D_l = D ; 
D_3 = D ;
D_4 = D ;

% State space representation
sys1 = ss(A_l1,B_l1,C_l1,D_l);
sys3 = ss(A_l3,B_l3,C_l3,D_3);
sys4 = ss(A_l4,B_l4,C_l4,D_4);

% Checking the continuous response 
figure
initial(sys1,x_inLue)

figure
initial(sys3,x_inLue)

figure
initial(sys4,x_inLue)

% Checking the step response
figure
step(sys1)

figure
step(sys3)

figure
step(sys4)
grid on

% For output vector 4 we get the best Luenberger observer


