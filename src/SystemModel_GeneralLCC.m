function [ A,B ] = SystemModel_GeneralLCC( N,M,alpha1,alpha2,alpha3 )
% Generate LTI system model for LCC
% Input:
%       n: number of the following HDVs
%       m: number of the preceding HDVs
%       alpha1, alpha2, alpha3: linearized coefficients
% Output:
%       A, B: system matrices

A1 = [0,-1;alpha1,-alpha2];
A2 = [0,1;0,alpha3];


A = zeros(2*N+2*M+2,2*N+2*M+2);
B = zeros(2*N+2*M+2,1);


A(1:2,1:2) = A1;

for i=2:(M+N+1)
    A(2*i-1:2*i,2*i-1:2*i) = A1;
    A(2*i-1:2*i,2*i-3:2*i-2) = A2;
end

B(2*M+2) = 1;

A(2*M+2,:) = 0;

end

