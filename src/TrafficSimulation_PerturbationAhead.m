% =========================================================================
%          Traffic Simulation when One Perturbation Happens Ahead
% Scenario:
%       There are m HDVs ahead of the CAV and n HDVs behind the CAV
%       There also exists a head vehicle at the very beginning
%       One slight disturbance happens at the head vehicle
%
% See Section V.A of the following paper for details
%   Title : Leading Cruise Control in Mixed Traffic Flow:
%                      System Modeling,Controllability,and String Stability
%   Author:Jiawei Wang, Yang Zheng, Chaoyi Chen, Qing Xu and Keqiang Li
% =========================================================================

clc; clear; close all;

% -------------------------------------------------------------------------
%   Parameter setup
%--------------------------------------------------------------------------
m = 2;                  % number of preceding vehicles
n = 2;                  % number of following vehicles
PerturbedID = 0;        % perturbation on vehicle
                        % 0. Head vehicle
                        % 1 - m. Preceding vehicles
                        % m+2 - n+m+1. Following vehicles
PerturbedType = 1;      % perturbation type
                        % 1:Sine-wave Perturbation;  2: Braking
mix = 1;                % Mix traffic or all HDVs

% ------------------------------------------
% Connectivity pattern
% ------------------------------------------
connectivityType = 1;               % Different connectivity patterns

K = [1,-1,1,-1,0,0,-1,-1,-1,-1];    % Feedback gain

switch connectivityType
    case 1
        K(3:end) = 0;
    case 2
        K(5:end) = 0;
    case 3 
        K(9:end) = 0;
    case 4
        %
end

% ------------------------------------------
% Parameters in the car-following model
% ------------------------------------------
alpha = 0.6; % Driver Model: OVM
beta  = 0.9;
s_st  = 5;
s_go  = 35;

% Traffic equilibrium
v_star   = 15;   % Equilibrium velocity
acel_max = 2;
dcel_max = -5;
v_max    = 30;
s_star   = acos(1-v_star/v_max*2)/pi*(s_go-s_st)+s_st; % Equilibrium spacing

% linearized model
alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
alpha2 = alpha+beta;
alpha3 = beta;

% Simulation length
TotalTime = 100;
Tstep     = 0.01;
NumStep   = TotalTime/Tstep;

% ------------------------------------------------------------------------
% Some output information
% ------------------------------------------------------------------------
fprintf('============================================================\n')
fprintf('    Traffic Simulation when One Perturbation Happens Ahead \n')
fprintf('          By Jiawei Wang, Yang Zheng \n')
fprintf('============================================================\n')

fprintf('Number of HDV vehicles behind: %d\n',n)
fprintf('Number of HDV vehicles ahead : %d\n',m)
fprintf('Perturbation vehicle Id      : %d\n',PerturbedID)
fprintf('---------------------------\n')
fprintf('HDV car-following model: optimal velocity model (OVM) \n')
fprintf('Parameter setup in HDV car-following model: \n')
fprintf('    alpha  beta  s_st  s_go  v_max \n    %4.2f  %4.2f  %4.2f  %4.2f  %4.2f\n',alpha,beta,s_st,s_go,v_max)
fprintf('Coefficients in linearized HDV car-following model: \n')
fprintf('    alpha1  alpha2  alpha3 \n    %4.2f    %4.2f    %4.2f \n',alpha1,alpha2,alpha3)
fprintf('---------------------------\n')
fprintf('Feedback gain of the controller:\n')
fprintf('mu_{-2}  k_{-2}  mu_{-1}  k_{-1}  mu_{1}  k_{1}  mu_{2}  k_{2}\n')
fprintf('%4.2f    %4.2f    %4.2f    %4.2f    %4.2f    %4.2f    %4.2f    %4.2f\n',K(1),K(2),K(3),K(4),K(6),K(7),K(8),K(9))
fprintf('---------------------------\n')
fprintf('Simulation length (time step): %d  (%4.2f)\n',TotalTime,Tstep)  % this can be improved
fprintf('-----------------------------------------------------------\n')
fprintf('   Simulation beigns ...')

% ------------------------------------------------------------------------
% Traffic simulation
% ------------------------------------------------------------------------

switch mix
    case 1
        % When will the controller work. 0:Controller Work; Large: won't work
        ActuationTime = 0;
    case 0
        ActuationTime = 99999;
end

% -----------------------------------------------
% Define state variables
% -----------------------------------------------
%Initial State for each vehicle
S = zeros(NumStep,m+n+2,3);
dev_s = 0;
dev_v = 0;
co_v = 1.0;
v_ini = co_v*v_star; %Initial velocity
%from -dev to dev
S(1,:,1) = linspace(0,-(m+n+1)*s_star,m+n+2)'+(rand(m+n+2,1)*2*dev_s-dev_s);
%The vehicles are uniformly distributed on the ring road with a random deviation
S(1,:,2) = v_ini*ones(m+n+2,1)+(rand(m+n+2,1)*2*dev_v-dev_v);

% meaning of parameters
% 1:head vehicle
% 2~(m+1): preceding vehicles
% m+2:CAV
% (m+3)~(m+n+2): following vehicles
ID = zeros(1,m+n+2);
if mix
    ID(m+2) = 1;
end

X = zeros(2*(m+n+1),NumStep);
u = zeros(NumStep,1);
V_diff = zeros(NumStep,m+n+1);  %Velocity Difference
D_diff = zeros(NumStep,m+n+1);  %Following Distance

% ---------------------------------------------------------
% Simulation starts here
% ---------------------------------------------------------
tic
for k = 1:NumStep-1
    %Update acceleration
    V_diff(k,:) = S(k,1:(end-1),2)-S(k,2:end,2);
    D_diff(k,:) = S(k,1:(end-1),1)-S(k,2:end,1);
    cal_D = D_diff(k,:); %For the boundary of Optimal Veloicity Calculation
    for i = 1:m+n+1
        if cal_D(i)>s_go
            cal_D(i) = s_go;
        elseif cal_D(i)<s_st
            cal_D(i) = s_st;
        end
    end
    
    %OVM Model
    %V_d = v_max/2*(1-cos(pi*(h-h_st)/(h_go-h_st)));
    %a2 = alpha*(V_h-v2)+beta*(v1-v2);
    acel = alpha*(v_max/2*(1-cos(pi*(cal_D-s_st)/(s_go-s_st)))-S(k,2:end,2))+beta*V_diff(k,:);
    acel(acel>acel_max)=acel_max;
    acel(acel<dcel_max)=dcel_max;
    % SD as ADAS to prevent crash
    acel_sd = (S(k,2:end,2).^2-S(k,1:(end-1),2).^2)./2./D_diff(k,:);
    acel(acel_sd>abs(dcel_max)) = dcel_max;
    
    S(k,2:end,3) = acel;
    % the preceding vehicle
    S(k,1,3) = 0;
    
    % Perturbation
    switch PerturbedType
        case 1
            P_A = 0.2;
            P_T = 12;
            if k*Tstep>20 && k*Tstep<20+P_T
                S(k,PerturbedID+1,3)=P_A*cos(2*pi/P_T*(k*Tstep-20));
            end
        case 2
            if (k*Tstep>20)&&(k*Tstep<21)
                S(k,PerturbedID+1,3)=5;
            end
        case 3
            if k*Tstep == 20
                S(k,PerturbedID+1,2) = v_star - 0.4;
                
            end
    end
    
    X(1:2:end,k) = reshape(D_diff(k,:),m+n+1,1) - s_star;
    X(2:2:end,k) = reshape(S(k,2:end,2),m+n+1,1) - v_star;
    if k > ActuationTime/Tstep
        u(k) = K*X(:,k);
        if u(k) > acel_max
            u(k) = acel_max;
        elseif u(k) < dcel_max
            u(k) = dcel_max;
        end
        S(k,m+2,3) = S(k,m+2,3) + u(k);
    end
    
    
    S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
    S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
    
    
end
tsim = toc;

fprintf('  ends at %6.4f seconds \n', tsim);

% ------------------------------------------------------------------------
%  Plot the results
% ------------------------------------------------------------------------
fprintf('-----------------------------------------------------------\n')
fprintf('    Now plot the velocity profiles for demonstration, please wait ... \n')

Wsize = 22;
figure;

i = 1;
p1 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',2,'Color',[190 190 190]/255);
hold on;
for i=2:(m+1)
    p2 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',2,'Color',[90 90 90]/255);
end
i = m+2;
p3 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',2,'Color',[244, 53, 124]/255);
for i=(m+3):(n+m+2)
    p4 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',2,'Color',[67, 121, 227]/255);
end

set(gca,'TickLabelInterpreter','latex','fontsize',Wsize-4);
grid on;
xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',Wsize,'Interpreter','latex','Color','k');
yl = ylabel('Velocity [$\mathrm{m/s}$]','fontsize',Wsize,'Interpreter','latex','Color','k');

set(gca,'xlim',[20,45]);

switch PerturbedType
    case 1 
        set(gca,'ylim',[14.5,15.5]);
    case 2
        set(gca,'ylim',[0,30]);
    case 3
        set(gca,'ylim',[14.5,15.5]);
        set(gca,'xlim',[20,30]);
end

if m == 0
    l = legend([p1 p3 p4],'Head vehicle','CAV','HDVs behind','Location','NorthEast');
elseif n == 0
    
else
l = legend([p1 p2 p3 p4],'Head vehicle','HDVs ahead','CAV','HDVs behind','Location','NorthEast');
end

l.Position=[0.5,0.6,0.4,0.3];

set(gcf,'Position',[250 150 480 350]);
fig = gcf;
fig.PaperPositionMode = 'auto';
set(l,'fontsize',Wsize-4,'box','off','Interpreter','latex');
% print(gcf,['..\Figures\Simulation_PerturbationAhead_Controller_',num2str(controllerType)],'-painters','-depsc2','-r300');

