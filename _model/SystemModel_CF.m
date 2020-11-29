function [ A,B ] = SystemModel_CF( N,alpha1,alpha2,alpha3 )
% Linear Car-following LCC model
% Input
%    N: the number of vehicles in CF-LCC
%    alpha1, alpha2, alpha3: parameters from the linearized car-following model
% Output
%    state-space model: [A, B]
%                       \dot{x} = Ax + Bu
%
%   See Section II of the following paper for modeling details
%   Title : Leading Cruise Control in Mixed Traffic Flow:
%                      System Modeling,Controllability,and String Stability
%   Author:Jiawei Wang, Yang Zheng, Chaoyi Chen, Qing Xu and Keqiang Li

    A = zeros(2*N+2,2*N+2);
    B = zeros(2*N+2,1);

    A1 = [0,-1;alpha1,-alpha2];
    A2 = [0,1;0,alpha3];
    A(1:2,1:2) = A1;
    for i=2:(N+1)
        A(2*i-1:2*i,2*i-1:2*i)   = A1;
        A(2*i-1:2*i,2*i-3:2*i-2) = A2;
    end
    B(2) = 1;
end

