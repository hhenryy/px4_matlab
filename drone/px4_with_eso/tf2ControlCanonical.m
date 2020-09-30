TF = G__Omega_PX4


[b,a] = tfdata(TF,'v');


A = diag(ones(length(a)-2,1),1);
A(end,:) = -a(2:end);

D = b(1);
C = zeros(1,length(b)-1);


for c=1:length(b)-1
    
    b(end-c+1)
    C(c) = b(end-c+1) - a(end-c+1)*b(c);
    
    
    
end
