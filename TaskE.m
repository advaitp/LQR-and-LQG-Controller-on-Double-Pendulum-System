clc
load("param.mat");

% Condition 1
C1 = [1 0 0 0 0 0];

% Condition 2
C2 = [0 0 1 0 0 0;
      0 0 0 0 1 0];

% Condition 3
C3 = [1 0 0 0 0 0;
      0 0 0 0 1 0];

% Condition 4
C4 = [1 0 0 0 0 0;
      0 0 1 0 0 0;  
      0 0 0 0 1 0];

% Observability matrix 
% st1 = [C1 ;C1*A; C1*(A^2); C1*(A^3); C1*(A^4); C1*(A^5)];
% st2 = [C2; C2*A; C2*(A^2); C2*(A^3); C2*(A^4); C2*(A^5)];
% st3 = [C3; C3*A; C3*(A^2); C3*(A^3); C3*(A^4); C3*(A^5)];
% st4 = [C4; C4*A; C4*(A^2); C4*(A^3); C4*(A^4); C4*(A^5)];

st1 = obsv(A,C1);
st2 = obsv(A,C2);
st3 = obsv(A,C3);
st4 = obsv(A,C4);

disp('Rank of matrix st1');
r1 = rank(st1);
disp(r1);
if(r1 == 6) 
    disp('The output vector for state x is observable');
else
    disp('The output vector for state x is not observable');
end

disp('Rank of matrix st2');
r2 = rank(st2);
disp(r2);
if(r2 == 6) 
    disp('The output vector for state theta1 and theta2 is observable');
else
    disp('The output vector for state theta1 and theta2 is not observable');
end

disp('Rank of matrix st3');
r3 = rank(st3);
disp(r3);
if(r3 == 6) 
    disp('The output vector for state x and theta2 is observable');
else
    disp('The output vector for state x and theta2 is not observable');
end

disp('Rank of matrix st4');
r4 = rank(st4);
disp(r4);
if(r4 == 6) 
    disp('The output vector for state x, theta1 and theta2 is observable');
else
    disp('The output vector for state x, theta1 and theta2 is not observable');
end


