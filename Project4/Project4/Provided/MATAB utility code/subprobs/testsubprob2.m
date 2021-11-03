clear err1 err2

N=1000;

for i=1:N

k1=rand(3,1);k1=k1/norm(k1);
k2=rand(3,1);k2=k2/norm(k2);
theta1=(rand-.5)*2*pi;
theta2=(rand-.5)*2*pi;
p1=randn(3,1);
p2=rot(k2,-theta2)*rot(k1,theta1)*p1;

[q1,q2]=subprob2(k1,k2,p1,p2);

err1(i)=min([norm(q1(1)-theta1) norm(q1(2)-theta1)]);
err2(i)=min([norm(q2(1)-theta2) norm(q2(2)-theta2)]);

if max([err1(i) err2(i)])>eps*2000000;
    disp(sprintf('| theta 1 error | = %0.5g',err1(i)));
    disp(sprintf('| theta 2 error | = %0.5g',err2(i)));
    error('mismatch!');
    return;
end

end

disp(['subproblem 2 checked out after ',num2str(N),' runs']);
disp(sprintf('theta 1 error norm = %0.5g',norm(err1)));
disp(sprintf('theta 2 error norm = %0.5g',norm(err2)));
figure(1);plot((1:N),err1,'x');
figure(2);plot((1:N),err2,'x');

