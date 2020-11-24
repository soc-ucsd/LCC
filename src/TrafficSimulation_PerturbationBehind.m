%% Discription
% Consider the case of CF-LCC or FD-LCC.
% One CAV is leading the motion of n HDVs behind.
% One sudden disturbance happens at one HDV behind the CAV.
% The animation can be found in the "demo" folder.

%%
clc;
clear;
close all;

%%%%%%%%%%%%%%%%%%
FD_bool = 1; % 0. CF-LCC; 1. FD-LCC
%%%%%%%%%%%%%%%%%%

m = 0; % number of preceding vehicles
n = 10; % number of following vehicles


PerturbedID = 2;
% 0. Head vehicle
% 1 ~ m. Preceding vehicles
% m+2 ~ n+m+1. Following vehicles

% the PerturbedID-th HDV is under the perturbation
PerturbedType = 2;
% 1:Sine Perturbation  2:Brake   3: Initial Deviation

% Mix or not
mix = 1;

switch mix
    case 1
        ActuationTime = 0;
    case 0
        ActuationTime = 99999;
end
%When will the controller work. 0:Controller Work; Large: won't work

%% Parameters

v_star = 15;

acel_max = 2;
dcel_max = -5;


v_max = 30;
TotalTime = 100;
Tstep = 0.01;
NumStep = TotalTime/Tstep;

%Driver Model: OVM
alpha = 0.6;
beta = 0.9;
s_st = 5;
s_go = 35;

%Equilibrium
s_star = acos(1-v_star/v_max*2)/pi*(s_go-s_st)+s_st;


alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
alpha2 = alpha+beta;
alpha3 = beta;



%% Define state variables

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


%%%%%%%%%%%%%%%%%%%%%%%%

% 1:head vehicle
% 2~(m+1): preceding vehicles
% m+2:CAV
% (m+3)~(m+n+2): following vehicles

ID = zeros(1,m+n+2);
if mix
    ID(m+2) = 1;
end
%%%%%%%%%%%%%%%%%%%%%%%%


X = zeros(2*(m+n+1),NumStep);
%0. HDV  1. CAV
u = zeros(NumStep,1);

%Velocity Difference
V_diff = zeros(NumStep,m+n+1);
%Following Distance
D_diff = zeros(NumStep,m+n+1);

%% Controller design

K = zeros(1,2*(n+1));

if FD_bool
    K(1:6) = [0,-0.5,-0.2,0.05,-0.1,0.05];
else
    K(1:6) = [0.1,-0.5,-0.2,0.05,-0.1,0.05];
end
   

%% Simulation begins

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
            P_T = 15;
            if k*Tstep>20 && k*Tstep<20+P_T
                S(k,PerturbedID+1,3)=P_A*cos(2*pi/P_T*(k*Tstep-20));
            end
        case 2
            if (k*Tstep>20)&&(k*Tstep<21)
                S(k,PerturbedID+1,3)=-5;
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
        S(k,m+2,3) = u(k);
    end
    
    
    S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
    S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
    
    
end





%% Plot the velocity

Wsize = 22;
figure;

i = 1;
p1 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',2,'Color',[190 190 190]/255);
hold on;
for i=2:(m+1)
    p2 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',2,'Color','k');
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
        set(gca,'ylim',[10,20]);
        set(gca,'xlim',[20,50]);
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

l.Position=[0.52,0.6,0.4,0.3];


set(gcf,'Position',[250 150 480 350]);
fig = gcf;
fig.PaperPositionMode = 'auto';
set(l,'fontsize',Wsize-4,'box','off','Interpreter','latex');


