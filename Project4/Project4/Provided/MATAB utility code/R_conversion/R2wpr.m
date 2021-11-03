%
% R2wpr.m
%
% converts R in SO(3) to yaw-pitch-roll
% (based on Karel convention)
%

function [w,p,r]=R2wpr(R)
  
  w=atan2(-R(2,3),R(3,3));
  p=atan2(R(1,3),sqrt(R(2,3)^2+R(3,3)^2));
  r=atan2(-R(1,2),R(1,1));
  