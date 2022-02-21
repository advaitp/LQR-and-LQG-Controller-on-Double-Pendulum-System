function pendcart(x,u)
% Sx1 = sin(x(3));
% Cx1 = cos(x(3));
% 
% Sx2 = sin(x(5));
% Cx2 = cos(x(5));

% D = m*L*L*(M+m*(1-Cx^2));

dx(1) = x(2); 
dx(2)=(F-(g/2)*(m1*sin(2*x(3))+m2*sin(2*x(5)))-(m1*l1*(x(4)^2)*sin(x(3)))-(m2*l2*(x(6)^2)*sin(x(5))))/(M+m1*((sin(x(3)))^2)+m2*((sin(x(5)))^2));
dx(3)= x(4); 
dx(4)= (dxdt(2)*cos(x(3))-g*(sin(x(3))))/l1; 
dx(5)= x(6); 
dx(6)= (dxdt(2)*cos(x(5))-g*(sin(x(5))))/l2; ;


end()