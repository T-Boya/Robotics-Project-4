%
% sigma2R.m
%
% converts stereographic projection sigma=q/(1-qo) to R in SO(3)
%

function R=sigma2R(sigma)
  
ss=norm(sigma);
qo=(ss^2-1)/(ss^2+1);
R=eye(3,3)+2*qo*(1-qo)*crossmat(sigma)+...
    2*(1-qo)^2*crossmat(sigma)^2;
  