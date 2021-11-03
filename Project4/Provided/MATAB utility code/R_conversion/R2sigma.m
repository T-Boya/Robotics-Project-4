%
% R2sigma.m
%
% converts R in SO(3) to stereographic projection sigma=q/(1-qo)
%

function sigma=R2sigma(R)
  
  sigma=zeros(3,1);
  trR=trace(R);
  if abs(trR+1)<1e-5
    [sigma,th]=R2kth(R);
  else
    sigma=vee(R-R')/(-2*sqrt(trR+1)-(trR+1));
  end
  