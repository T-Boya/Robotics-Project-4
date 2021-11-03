%
% ypr2R.m
%
% converts yaw-pitch-roll to R in SO(3)
%

function R=rpy2R(y,p,r)
  
  z0=[0;0;1];y0=[0;1;0];x0=[1;0;0];
  R = expm(crossmat(z0)*y)*...
      expm(crossmat(y0)*p)*...
      expm(crossmat(x0)*r);
  