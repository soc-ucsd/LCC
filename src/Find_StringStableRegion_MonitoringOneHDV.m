%% Description
% Find the head-to-tail string stable region when monitoring one HDV

clc;
clear;
close all;

%% Parameters


% number of the following HDVs
n = 2;
% number of the preceding HDVs
m = 2;

DriverDynamics = 1;


if m ~= 0 && n ~= 0
    ID = zeros(1,m+n);
    ID(1:m) = -m:-1;
    ID(m+1:m+n) = 1:n;
elseif m == 0
    ID = 1:n;
elseif n == 0
    ID = -m:-1;
end

% id of the HDV that has feedback
for id = ID
    
    K = -10:0.1:10;
    Mu = -10:0.1:10;
    
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
    
    %%
    h=waitbar(0,'please wait');
    iTest = 1;
    TestNumber = length(K)*length(Mu);
    
    for ik0 = 1:length(K)
        for ik1 = 1:length(Mu)
            k = K(ik0);
            mu = Mu(ik1);
            HDV_Tf = @(w)(-abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^n));
            
            if id<0
                minus_HeadTail_Tf = @(w)(-abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m))...
                    *abs((alpha3*1i*w+alpha1...
                    +(mu*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu+k*1i*w)...
                    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(id+1))...
                    /(-w^2+alpha1+1i*w*alpha2)));
                HeadTail_Tf = @(w)(abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m))...
                    *abs((alpha3*1i*w+alpha1...
                    +(mu*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu+k*1i*w)...
                    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(id+1))...
                    /(-w^2+alpha1+1i*w*alpha2)));
            else
                minus_HeadTail_Tf = @(w)(-abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m))...
                    *abs((alpha3*1i*w+alpha1)/(-w^2+alpha1+1i*w*alpha2...
                    -(mu*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu+k*1i*w)...
                    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^id)));
                HeadTail_Tf = @(w)(abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m))...
                    *abs((alpha3*1i*w+alpha1)/(-w^2+alpha1+1i*w*alpha2...
                    -(mu*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu+k*1i*w)...
                    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^id)));
            end
            
            [x,fval] = fminbnd(minus_HeadTail_Tf,1e-8,100);
            fval = -fval;
            if fval<=1
                
                SS_bool(ik0,ik1) = 1;
                
            end
            str=['DriverDynamics=',num2str(DriverDynamics),'; ID=',num2str(id),...
                '; Processing...',num2str(iTest/TestNumber*100),'%'];
            waitbar(iTest/TestNumber,h,str);
            iTest = iTest+1;
        end
    end
    
    close(h);
    
    save(['..\_data\',date,'_SSRegion_n_',num2str(n),'_m_',num2str(m),...
        '_DriverDynamics_',num2str(DriverDynamics),'_FeedbackID_',num2str(id),'.mat']);
    
end
