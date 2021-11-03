function khat = crossmat(k)
  
if length(k)==3
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
elseif length(k)==6
  khat=[[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0] k(4:6);zeros(1,4)];
else
    khat=[];
    disp('input vector is of wrong dimension! ');
end
