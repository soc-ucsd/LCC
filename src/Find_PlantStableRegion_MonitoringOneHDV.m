%% Description
% Find the plant stable region when monitoring one HDV

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
    
    
    %%
    
    %%%%% Please Change %%%%%
    switch DriverDynamics
        case 1
            alpha = 0.6;
            beta = 0.9;
        case 2
            alpha = 0.4;
            beta = 0.6;
    end
    
    %%%%% No change %%%%%
    v_max  = 30;
    s_st   = 5;
    s_go   = 35;
    alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
    alpha2 = alpha+beta;
    alpha3 = beta;
    
    [A,B] = SystemModel_GeneralLCC(n,m,alpha1,alpha2,alpha3);
    
    PS_bool = zeros(length(K),length(Mu));
    
    %%
    h=waitbar(0,'please wait');
    iTest = 1;
    TestNumber = length(K)*length(Mu);
    
    for ik0 = 1:length(K)
        for ik1 = 1:length(Mu)
            k = K(ik0);
            mu = Mu(ik1);
            
            Feedback = zeros(1,2*(m+n+1));
            
            Feedback(2*m+1) = alpha1;
            Feedback(2*m+2) = -alpha2;
            Feedback(2*m) = alpha3;
            
            Feedback(2*(m+id)+1) = Feedback(2*(m+id)+1)+k;
            Feedback(2*(m+id)+2) = Feedback(2*(m+id)+2)+mu;
            
            A_closed = A+B*Feedback;
            
            
            
            
            if isempty(find(real(eig(A_closed))>0,1))
                
                PS_bool(ik0,ik1) = 1;
                
            end
            str=['DriverDynamics=',num2str(DriverDynamics),'; ID=',num2str(id),...
                '; Processing...',num2str(iTest/TestNumber*100),'%'];
            waitbar(iTest/TestNumber,h,str);
            iTest = iTest+1;
        end
    end
    
    close(h);
    
%     save(['..\_data\',date,'_PSRegion_n_',num2str(n),'_m_',num2str(m),...
%         '_DriverDynamics_',num2str(DriverDynamics),'_FeedbackID_',num2str(id),'.mat']);
    
end



