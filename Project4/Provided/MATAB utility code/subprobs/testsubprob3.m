clear err

N=1000;

for i=1:N

k=rand(3,1);k=k/norm(k);
q=(rand-.5)*2*pi;
p1=randn(3,1);
p2=randn(3,1);

d=norm(p2-rot(k,q)*p1);

th=subprob3(k,p1,p2,d);

err(i)=min([abs(th(1)-q) abs(th(2)-q) ...
    abs(th(1)-q+2*pi) abs(th(2)-q+2*pi) ...    
    abs(th(1)-q-2*pi) abs(th(2)-q-2*pi) ...    
    ]);

if err(i)>eps*2000000;
    disp(sprintf('| theta error | = %0.5g',err(i)));
    error('mismatch!');
    return;
end

end

disp(['subproblem 3 checked out after ',num2str(N),' runs']);
disp(sprintf('theta error norm = %0.5g',norm(err)));
figure(1);plot((1:N),err,'x');


