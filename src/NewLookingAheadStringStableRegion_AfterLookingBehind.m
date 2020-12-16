% =========================================================================
%   New "Looking Ahead" String Stable Region of LCC after Looking Behind
%
% Find the new "looking ahead" head-to-tail string stable regions after
% incorporating the motion of one vehicle behind
%
% See Section IV.B (Fig. 7(c)-(f)) of the following paper for details
%   Title : Leading Cruise Control in Mixed Traffic Flow:
%                      System Modeling,Controllability,and String Stability
%   Author:Jiawei Wang, Yang Zheng, Chaoyi Chen, Qing Xu and Keqiang Li
% =========================================================================

clc;clear; close all;

% -------------------------------------------------------------------------
%   Parameter setup
% -------------------------------------------------------------------------
id_ahead = -2;          % id of the preceding HDV which is under 
                        % investigation of the string stability region
id_behind = 2;          % id of the following HDV which is incorporated

m = 2;                  % number of preceding vehicles
n = 2;                  % number of following vehicles

generate_data_bool = 0; % 0. Use existing data; 1. Generate new data

% ------------------------------------------
% Feedback gain setup for the following HDV
% ------------------------------------------
if id_behind == 1
    mu_behind = -1;
    k_behind = -1;
elseif id_behind == 2
    mu_behind = -1;
    k_behind = -1;
end

% ------------------------------------------
% Parameters in the car-following model
% ------------------------------------------
s_star = 20; % Equilibrium spacing
DriverDynamics = 1;
switch DriverDynamics
    case 1
        alpha = 0.6;
        beta = 0.9;
    case 2
        alpha = 0.4;
        beta = 0.6;                               
end
v_max  = 30;
s_st   = 5;
s_go   = 35;
alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
alpha2 = alpha+beta;
alpha3 = beta;

% -------------------------------------------------------------------------
% Some output information
% -------------------------------------------------------------------------
fprintf('=====================================================================\n')
fprintf('New "Looking Ahead" String Stable Region of LCC after Looking Behind \n')
fprintf('                 By Jiawei Wang, Yang Zheng \n')
fprintf('=====================================================================\n')
fprintf('Number of HDV vehicles ahead: %d\n',m)
fprintf('Number of HDV vehicles behind: %d\n',n)
fprintf('---------------------------\n')
fprintf('HDV car-following model: optimal velocity model (OVM) \n')
fprintf('Parameter setup in HDV car-following model: \n')
fprintf('    alpha  beta  s_st  s_go  v_max \n    %4.2f  %4.2f  %4.2f  %4.2f  %4.2f\n',alpha,beta,s_st,s_go,v_max)
fprintf('Coefficients in linearized HDV car-following model: \n')
fprintf('    alpha1  alpha2  alpha3 \n    %4.2f    %4.2f    %4.2f \n',alpha1,alpha2,alpha3)
fprintf('---------------------------\n')
fprintf('Index of the HDV ahead under investigation: %d \n',id_ahead)
fprintf('---------------------------\n')
fprintf('Index of the HDV behind under incorporation: %d \n',id_behind)
fprintf('Fixed "looking-behind" feedback gain: \n    mu = %4.2f, k = %4.2f \n',mu_behind,k_behind)
fprintf('-----------------------------------------------------------\n')

if generate_data_bool
    fprintf('    Now calculate the head-to-tail string stable region for "looking-ahead" feedback policies\n')
    fprintf('    Please wait ... \n')
else
    fprintf('    Now skip the calculation for head-to-tail string stable regions for "looking-ahead" feedback policies\n')
end

% ------------------------------------------------------------------------
% Experiment starts here
% Find the new "looking ahead" head-to-tail string stable region
% ------------------------------------------------------------------------
if generate_data_bool
    % ------------------------------------------
    % Range of the investigated feedback gain
    % ------------------------------------------
    K = -10:0.02:10;
    Mu = -10:0.02:10;
    
    % Whether this setup is string stable
    SS_bool = zeros(length(K),length(Mu));
    
    h=waitbar(0,'please wait');
    iTest = 1;
    TestNumber = length(K)*length(Mu);
    
    for ik0 = 1:length(K)
        parfor ik1 = 1:length(Mu)
            k = K(ik0);
            mu = Mu(ik1);
            
            % ------------------------------------------
            % Judge whether this feedback setup is head-to-tail string stable
            % ------------------------------------------
            % Calculate the transfer function
            minus_HeadTail_Tf = @(w)(-abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m))...
                *abs((alpha3*1i*w+alpha1...
                +(mu*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu+k*1i*w)...
                *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(id_ahead+1))...
                /(-w^2+alpha1+1i*w*alpha2...
                -(mu_behind*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_behind+k_behind*1i*w)...
                *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^id_behind)));
            [x,fval] = fminbnd(minus_HeadTail_Tf,1e-8,100);
            fval = -fval;
            % Judge whether the maximum magnitude of the transfer function 
            % is beneath one
            if fval<=1                
                SS_bool(ik0,ik1) = 1;                
            end
            
        end
        str=['DriverDynamics=',num2str(DriverDynamics),'; ID=',num2str(id_ahead),...
            '; Processing...',num2str(ik0/length(K)*100),'%'];
        waitbar(ik0/length(K),h,str);
        
    end
    
    close(h);
    
    save(['..\_data\',date,'_SSRegion_WithLookingBehind_id_behind_',num2str(id_behind),...
        '_mu_behind_',num2str(mu_behind),'_k_behind_',num2str(k_behind)...
        '_n_',num2str(n),'_m_',num2str(m),...
        '_DriverDynamics_',num2str(DriverDynamics),'_FeedbackID_',num2str(id_ahead),'.mat']);

end

% ------------------------------------------
% Data for original "looking ahead" string stable region
% ------------------------------------------
load(['..\_data\30-Mar-2020_SSRegion_n_',num2str(n),'_m_',num2str(m),...
    '_DriverDynamics_',num2str(DriverDynamics),'_FeedbackID_',num2str(id_ahead),'.mat']);
OriginalSS_bool = SS_bool;

% ------------------------------------------
% Data for new "looking ahead" string stable region after looking behind
% ------------------------------------------
if ~generate_data_bool
    load(['..\_data\30-Mar-2020_SSRegion_WithLookingBehind_id_behind_',num2str(id_behind),...
        '_mu_behind_',num2str(mu_behind),'_k_behind_',num2str(k_behind)...
        '_n_',num2str(n),'_m_',num2str(m),...
        '_DriverDynamics_',num2str(DriverDynamics),'_FeedbackID_',num2str(id_ahead),'.mat']);
    FinalSS_bool = OriginalSS_bool+SS_bool;
else
    % Utilize the newly generate data
    % The data name might need to be changed
    load(['..\_data\',date,'_SSRegion_WithLookingBehind_id_behind_',num2str(id_behind),...
        '_mu_behind_',num2str(mu_behind),'_k_behind_',num2str(k_behind)...
        '_n_',num2str(n),'_m_',num2str(m),...
        '_DriverDynamics_',num2str(DriverDynamics),'_FeedbackID_',num2str(id_ahead),'.mat']);
end




% ------------------------------------------------------------------------
%  Plot the String Stable Region
% ------------------------------------------------------------------------
fprintf('-----------------------------------------------------------\n')
if ~generate_data_bool
    fprintf('    Now plot the string stable region utilizing existing data, please wait ... \n')
else
    fprintf('    Now plot the string stable region utilizing newly generated data, please wait ... \n')
    
end

figure;

Wsize = 18;

[Mu_3d,K_3d] = meshgrid(K,Mu);

p = surf(K_3d,Mu_3d,FinalSS_bool);

mymap = [235,235,235;
    255, 181, 190;
    113, 178, 246]/255;
% Blue: original string stable region
% Red: expanded string stable region
% Gray: sting unstable region


colormap(mymap);

view([0 90]);

p.EdgeColor = 'none';

hold on;

set(gca,'TickLabelInterpreter','latex','fontsize',Wsize-2);
set(gca,'xlim',[min(K) max(K)]);
set(gca,'ylim',[min(Mu) max(Mu)]);

switch id_ahead
    case -2
        xlabel('$k_{-2}$','fontsize',Wsize,'Interpreter','latex','Color','k');
        ylabel('$\mu_{-2}$','fontsize',Wsize,'Interpreter','latex','Color','k');
    case -1
        xlabel('$k_{-1}$','fontsize',Wsize,'Interpreter','latex','Color','k');
        ylabel('$\mu_{-1}$','fontsize',Wsize,'Interpreter','latex','Color','k');
    case 1
        xlabel('$k_1$','fontsize',Wsize,'Interpreter','latex','Color','k');
        ylabel('$\mu_1$','fontsize',Wsize,'Interpreter','latex','Color','k');
    case 2
        xlabel('$k_2$','fontsize',Wsize,'Interpreter','latex','Color','k');
        ylabel('$\mu_2$','fontsize',Wsize,'Interpreter','latex','Color','k');
end

set(gcf,'Position',[250 150 350 280]);
fig = gcf;
fig.PaperPositionMode = 'auto';

% print(gcf,['../Figures/Fig4_Driver_',num2str(DriverDynamics),'_id_',num2str(id),'_BehindID_',num2str(id_behind)],'-depsc','-r300');


