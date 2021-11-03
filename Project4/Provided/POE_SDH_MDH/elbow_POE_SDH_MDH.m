%
% set up elbow arm in symbolic form
%

function [sdh, mdh] = elbow_POE_SDH_MDH(elbow)
L1=399.1;
L2=448;
L3=42;
L4=451;
L5=82;

% disp('T_{0T} from POE method');
% elbow

%SDH

% set up SDH parameters
elbow1.d=[L1;0;0;L4;0;L5];
elbow1.a=[0;L2;L3;0;0;0];
elbow1.alpha=sym([-pi/2;0;-pi/2;pi/2;-pi/2;0]);
elbow1.theta=[0;-pi/2;0;0;0;0];
% calculate T_{06}
elbow1=fwdkinsdh(elbow1);
% additional transformation to match with POE end effector frame
T6T=[[[0 1 0 ; 0 0 1; 1 0 0] zeros(3,1)];[zeros(1,3) 1]];
% This should match with POE's T_{0T}
T0T_SDH=simplify(elbow1.T*T6T);

% disp('T_{0T} from SDH method');
% disp(vpa(T0T_SDH));

% check
% disp('difference between POE and SDH forward kinematics');
% simplify(vpa(elbow-T0T_SDH))

%MDH

% set up MDH parameters
elbow2.d=[L1;0;0;L4;0;0;L5];
elbow2.a=[0;0;L2;L3;0;0;0]; % this is a_{i-1}
elbow2.alpha=sym([0;-pi/2;0;-pi/2;pi/2;-pi/2;0]);
elbow2.theta=[0;0;-pi/2;0;0;0;0];
% calculate T_{07}
elbow2=fwdkinmdh(elbow2);
% additional transformation to match with POE end effector frame
T7T=[[[0 1 0 ; 0 0 1; 1 0 0] zeros(3,1)];[zeros(1,3) 1]];
% This should match with POE's T_{0T|
T0T_MDH=simplify(elbow2.T*T7T);

% disp('T_{0T} from MDH method');
% disp(vpa(T0T_MDH));

% check

% disp('difference between POE and MDH forward kinematics');
% simplify(vpa(elbow-T0T_MDH))
sdh =vpa(elbow-T0T_SDH);
mdh = vpa(elbow-T0T_MDH);
end