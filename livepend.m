function livepend(y)
load("param.mat")
x = y(1);
theta1 = y(3);
theta2 = y(5);

% Dimensions of the system
W = 10;  % cart width
H = 4; % cart height
wr = 2.4;          % wheel radius
mr1 = 3;  % mass 1 radius
mr2 = 3;  % mass 1 radius

% positions
y = wr/2+H/2; % cart vertical position
pendx1 = x + l1*sin(theta1);
pendy1 = y - l1*cos(theta1);

pendx2 = x + l2*sin(theta2);
pendy2 = y - l2*cos(theta2);

plot([-100 100],[0 0],'k','LineWidth',1), hold on
rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[.5 0.5 1],'LineWidth',1.5); % Draw cart
rectangle('Position',[x-.9*W/2,0,wr,wr],'Curvature',1,'FaceColor',[0 0 0],'LineWidth',1.5); % Draw wheel
rectangle('Position',[x+.9*W/2-wr,0,wr,wr],'Curvature',1,'FaceColor',[0 0 0],'LineWidth',1.5); % Draw wheel
plot([x pendx1],[y pendy1],'k','LineWidth',1); % Draw pendulum1
rectangle('Position',[pendx1-mr1/2,pendy1-mr1/2,mr1,mr1],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);
plot([x pendx2],[y pendy2],'k','LineWidth',1); % Draw pendulum2
rectangle('Position',[pendx2-mr2/2,pendy2-mr2/2,mr2,mr2],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);

axis([-10 10 -35 15]);, axis equal
set(gcf,'Position',[100 100 1000 400])
drawnow, hold off