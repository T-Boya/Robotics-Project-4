%
% invkin_iterJ.m
%
% inverse kinematics using Jacobian iteration (planar)
%

function robot=invkin_iterJ(robot)
    N = 200;
    alpha = 0.2;
    weights = [repelem(0.5, 3) repelem(1, 3)];

    % define unit z vector
    ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];
    % target (R,p)
    qTd=atan2(robot.T(2,1),robot.T(1,1));
    pTd=robot.T(1:3,4);    

    % set up storage space
    q0=robot.q;
    n=length(q0); % # of joints
    q=zeros(n,N+1); 
    q(:,1)=q0; % output joint displacements
    pT=zeros(3,N+1); % output p
    qT=zeros(3,N+1); % output quaternion
    
    
    Tinitial = robot.T;

    % iterative update
    for i=1:N
        % forward kinematics
        robot.q=q(:,i);
        robot=nlinkfwdkin(robot);
        % qT(:,i)=atan2(robot.T(2,1),robot.T(1,1));
        pT(:,i)=robot.T(1:3,4);  
        % task space error: angular error and position error
        % qT(:,i) = R2kth(robot.T(1:3, 1:3) * Tinitial(1:3, 1:3)')'; % angular error option 1
        qT(:,i) = R2qv(robot.T(1:3, 1:3) * Tinitial(1:3, 1:3)'); % angular error option 2
        angular_error = 2 * qT(:,i); % angular error option 2
        position_error = pT(:,i)-pTd;
        dX = [ angular_error; position_error ] .* weights';
        % Jacobian update - note 10 times the gain for orientation        
        % qq=q(:,i)-alpha*pinv(robot.J)*dX;
        qq=q(:,i)-alpha*robot.J'*inv(robot.J*robot.J'+.01*eye(6,6))*dX;
        q(:,i+1)=(qq>pi).*(-2*pi+qq)+(qq<-pi).*(2*pi+qq)+(qq<pi).*(qq>-pi).*qq;
    end
    % final iteration
    robot.q=q(:,N+1);
    robot=nlinkfwdkin(robot);
end

