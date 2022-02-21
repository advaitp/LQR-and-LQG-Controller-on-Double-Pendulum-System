clc;
syms M m1 m2 l1 l2 g;

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
cont = [B A*B (A^2)*B (A^3)*B (A^4)*B (A^5)*B];

r = rank(cont);
disp("Controllability matrix rank =");
disp(r);

disp("Determinant of matrix = ");
detr = det(cont);
disp(detr);

% Determinant of controllability matrix is -(g^6*l1^2 - 2*g^6*l1*l2 + g^6*l2^2)/(M^6*l1^6*l2^6)
% Here we conclude that l1 = l2 then matrix is not invertible
% Here if mass of crane M is very large which is not possible in real life makes matrix not invertible
% Therefore l1 != l2 and mass M should not be very large otherwise system is not controllable 

