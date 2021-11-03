%
% wpr2R.m
%
% converts yaw-pitch-roll to R in SO(3) 
% (based on Karel convention)
%

function R=wpr2R(w,p,r)
  
  z0=[0;0;1];y0=[0;1;0];x0=[1;0;0];
  R = expm(crossmat(x0)*w)*...
      expm(crossmat(y0)*p)*...
      expm(crossmat(z0)*r);
  