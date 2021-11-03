%
% mini-project 4 initialization of variables
%
%% setup
clear all; close all; clc;

zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

% define ABB IRB 1200 robot

L1=399.1;
L2=448; %L2=350;
L3=42;
L4=451; %L4=351;
L5=82;

% P
p01=0*ex+L1*ez;p12=zz;p23=L2*ez;p34=L3*ez+L4*ex;p45=zz;p56=zz;p6T=L5*ex;

% H
h1=ez;h2=ey;h3=ey;h4=ex;h5=ey;h6=ex;

% 
irb1200.P=[p01 p12 p23 p34 p45 p56 p6T];
irb1200.P = irb1200.P / 1000; % may cause errors
irb1200.H=[h1 h2 h3 h4 h5 h6];
irb1200.joint_type=[0 0 0 0 0 0];

% % define collision body for abb 1200
radius=.01;
[irb1200_rbt,colLink]=defineRobot(irb1200,radius);

%% Question 1
% 1A: iterative jacobian
% see nlinkfwdkin;

% 1B: analytical expression J43
proj4SymbolicJacobian;
% all below parameters are in frame 3
h1 = (rot(h2,q2) * rot(h3,q3))' * ez;
h2 = rot(h3,q3)' * ey;
h3 = eye(3) * ey;
h4 = rot(h4,q4) * ex;
h5 = rot(h4,q4)* rot(h5,q5) * ey;
h6 = rot(h4,q4)* rot(h5,q5) * rot(h6,q6) *  ex;
p14 = (rot(h2,q2) * rot(h3,q3))' * zz + rot(h3,q3)' * L2*ez + eye(3) * L3*ez;
p24 = rot(h3,q3)' * L2*ez + eye(3) * L3*ez;
p34 = eye(3) * L3*ez;
[p44, p54, p64] = deal(repelem(0, 3)');
J43 = [ h1 h2 h3 h4 h5 h6; hat(h1)*p14 hat(h2)*p24 hat(h3)*p34 hat(h4)*p44 hat(h5)*p54 hat(h6)*p64];

% 1D: Jacobian Comparison
proj4JacobianComparison;


% 1C: find geometric jacobian
jacobianGeometric = geometricJacobian(irb1200_rbt, repelem(0, 6)', 'body7'); 

% Cleanup
%   Reset H values
h1=ez;h2=ey;h3=ey;h4=ex;h5=ey;h6=ex;
%   Reset P values
p01=0*ex+L1*ez;p12=zz;p23=L2*ez;p34=L3*ez+L4*ex;p45=zz;p56=zz;p6T=L5*ex;

%% Question 2
% 2A: Calculate Singularities
singularities = simplify(det(J4_3) == 0)


% 2B: Show Singularities
% replace symbolic values with real
L1=399.1 / 1000;
L2=448 / 1000; %L2=350;
L3=42 / 1000;
L4=451 / 1000; %L4=351;
L5=82 / 1000;

% boundary
figure
show(irb1200_rbt, [0, 0, atan(-L4/ L3), 0, 0, 0]' ,'collision','on');

% interior
figure
show(irb1200_rbt, [0, atan((-L4*cos(0)-L3*sin(0))/(-L4*sin(0)+L3*cos(0)+L2*cos(0))), 0, 0, 0, 0]' ,'collision','on');

% wrist
figure
show(irb1200_rbt, [0, 0, 0, 0, pi, 0]' ,'collision','on');

%% Question 3
% test iterative inverse kinematics solver invkin_iterJ
% to change error metric used, switch between which line is commented
% between lines 40 and 41 in inkin_iterJ
proj4kincheck;

%% Question 4

% load data from project 3
load ./Provided/S_sphere_path_uniform l pS
load ./proj3_q_values q

q = q(:,:,1);
min_jac = zeros(100, 1);
path_length = zeros(100, 1);

% calculate minimum Jacobian singular values
for i = 2:100
    bot = irb1200;
    bot.q = q(:,i);
    bot = nlinkfwdkin(bot);
    min_jac(i) = min(svd(bot.J));
    path_length(i) = norm(pS(i) - pS(i - 1));
end
% plot minimum Jacobian singular values
figure
plot(cumsum(path_length), min_jac);
xlabel('Path Length Traversed (mm)')
ylabel('Minimum Jacobian Singular Value')
title('Minimum Jacobian Singular Value vs Path Length Traversed')

%% inverse kinematics plotting (bonus)
load S_sphere_path
% convert to equal path length grid
diffS=vecnorm(diff(p_S')');
ls=[0 cumsum(diffS)];
lf=sum(diffS);
N=100;
l=(0:lf/N:lf);

pS=interp1(ls,p_S',l,'spline')';
% plot it out again with equal path length
figure(2);plot3(pS(1,:),pS(2,:),pS(3,:),'rx','linewidth',3);
xlabel('x');ylabel('y');zlabel('z');
hold on;
% 3d sphere
surf(X,Y,Z)
% make it transparent
alpha 0.4
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);

% find the end effector frame
pc=r*ez;
N=length(pS);
xT=zeros(3,N);zT=zeros(3,N);yT=zeros(3,N);
for i=1:N   
    xT(:,i)=(pS(:,i)-pc)/norm(pS(:,i)-pc);
    if i<N
        yT(:,i)=(pS(:,i+1)-pS(:,i));
    else
        yT(:,i)=yT(:,i-1);
    end
    yT(:,i)=yT(:,i)-yT(:,i)'*xT(:,i)*xT(:,i);
    yT(:,i)=yT(:,i)/norm(yT(:,i));
    zT(:,i)=cross(xT(:,i),yT(:,i));
    R=[xT(:,i) yT(:,i) zT(:,i)];
end

% 
% find all inverse kinematics solutions
%
for i=1:N
    % specify end effector SE(3) frame
    Td{i}=[[xT(:,i) yT(:,i) zT(:,i)] pS(:,i);[0 0 0 1]];
    irb1200.T=Td{i};
    irb1200=invkin_iterJ(irb1200);
    q(:,i)=irb1200.q(:,1);
    % check forward kinematics to make sure the IK solution is correct
    irb1200.q=q(:,i);
    irb1200=fwdkiniter(irb1200);
    T(:,:,i)=irb1200.T;
end

% visualize inverse kinematics
for i=1:N
    % show robot pose (ever 5 frames)
    if mod(i,5)==0
        figure(2);show(irb1200_rbt,q(:,i),'collision','on');
        view(150,10);
    end
end
hold off

%% supporting functions

% 
%
% R2q.m
%
% converts R in SO(3) to unit quaternion q, (q0,q_vec)
%

function q=R2q(R)
  
  q=zeros(4,1);
  q(1)=.5*sqrt(trace(R)+1);
  if abs(q(1))<1e-5
    [k,theta]=R2kth(R);
    q(2:4)=k;
  else
    q(2:4)=vee(R-R')/4/q(1);
  end
end

%
% hat.m (converting a vector into a skew-symmetric cross-product matrix
%
% khat = hat(k)
%

function khat = hat(k)
  
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];

end

%
% rot.m
%
% rotation matrix given axis and angle
%
function R=rot(k,theta)
  
  k=k/norm(k);
  R=eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);

end
