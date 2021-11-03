%
% R2ypr.m
%
% converts R in SO(3) to yaw-pitch-roll
%

function [w,p,r]=R2ypr(R)
  
  y=atan2(R(2,1),R(1,1));
  p=atan2(-R(3,1),sqrt(R(1,1)^2+R(2,1)^2));
  r=atan2(R(3,2),R(3,3));
  