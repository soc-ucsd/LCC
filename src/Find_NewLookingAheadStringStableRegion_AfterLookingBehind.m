% =========================================================================
%               New "Looking Ahead" LCC
%
% Find the new "looking ahead" head-to-tail string stable regions after 
% incorporating the motion of the vehicles behind 
%
% See Section IV.B (Fig. 7(c)-(f)) of the following paper for details
%   Title : Leading Cruise Control in Mixed Traffic Flow:
%                      System Modeling,Controllability,and String Stability
%   Author:Jiawei Wang, Yang Zheng, Chaoyi Chen, Qing Xu and Keqiang Li
% =========================================================================

clc;clear; close all;


%% Parameters

id_behind = 2;

DriverDynamics = 1;


% number of the following HDVs
n = 2;
% number of the preceding HDVs
m = 2;


if id_behind == 1
    
    mu_behind = -1;
    k_behind = -1;
    
elseif id_behind == 2
    
    mu_behind = -1;
    k_behind = -1;
end

save_data_bool = 1; 
% 0. Not save the data; 1. Save the data

% id of the HDV that has feedback
for id_ahead = -2:-1
    
    K = -10:0.02:10;
    Mu = -10:0.02:10;
    
    s_star = 20;
    
    
    %% OVM parameters
    
    
    switch DriverDynamics
        case 1
            alpha = 0.6;
            beta = 0.9;
        case 2
            alpha = 0.4;
            beta = 0.6;
    end
    
    % Other OVM parameters
    v_max  = 30;
    s_st   = 5;
    s_go   = 35;
    alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
    alpha2 = alpha+beta;
    alpha3 = beta;
    
    SS_bool = zeros(length(K),length(Mu));
    
    %% Find the string stable region
    h=waitbar(0,'please wait');
    iTest = 1;
    TestNumber = length(K)*length(Mu);
    
    for ik0 = 1:length(K)
        parfor ik1 = 1:length(Mu)
            k = K(ik0);
            mu = Mu(ik1);
            HDV_Tf = @(w)(-abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^n));
            

            
            minus_HeadTail_Tf = @(w)(-abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m))...
                *abs((alpha3*1i*w+alpha1...
                +(mu*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu+k*1i*w)...
                *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(id_ahead+1))...
                /(-w^2+alpha1+1i*w*alpha2...
                -(mu_behind*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_behind+k_behind*1i*w)...
                *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^id_behind)));
            

            
            [x,fval] = fminbnd(minus_HeadTail_Tf,1e-8,100);
            fval = -fval;
            if fval<=1
                
                SS_bool(ik0,ik1) = 1;
                
            end
            
        end
        str=['DriverDynamics=',num2str(DriverDynamics),'; ID=',num2str(id_ahead),...
            '; Processing...',num2str(ik0/length(K)*100),'%'];
        waitbar(ik0/length(K),h,str);
        
    end
    
    close(h);
    
    if save_data_bool
    save(['..\_data\',date,'_SSRegion_WithLookingBehind_id_behind_',num2str(id_behind),...
        '_mu_behind_',num2str(mu_behind),'_k_behind_',num2str(k_behind)...
        '_n_',num2str(n),'_m_',num2str(m),...
        '_DriverDynamics_',num2str(DriverDynamics),'_FeedbackID_',num2str(id_ahead),'.mat']);
    end
    
end