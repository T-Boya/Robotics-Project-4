%
% g2R.m
%
% converts Gibb's vector to R in SO(3)
%

function R=g2R(g)
  
  R=eye(3,3)+2*crossmat(g)*(eye(3,3)+crossmat(g))/(1+norm(g)^2);
  