function [ A,B ] = SystemModel_CF( N,alpha1,alpha2,alpha3 )

A1 = [0,-1;alpha1,-alpha2];
A2 = [0,1;0,alpha3];


A = zeros(2*N+2,2*N+2);
B = zeros(2*N+2,1);


A(1:2,1:2) = A1;



for i=2:(N+1)
    A(2*i-1:2*i,2*i-1:2*i) = A1;
    A(2*i-1:2*i,2*i-3:2*i-2) = A2;
end
B(2) = 1;

end

