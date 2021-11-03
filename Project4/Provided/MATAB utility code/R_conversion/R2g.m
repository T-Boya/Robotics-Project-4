%
% R2g.m
%
% converts R in SO(3) to the gibb's vector
%

function g=R2g(R)
  
  g=zeros(3,1);

  g=vee(R-R')/(1+trace(R));
  
  