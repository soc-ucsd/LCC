% =========================================================================
%               Control Energy of Free-Driving LCC
%
% Numerically calculate the three energy-related metrics of the FD-LCC 
% system at different system sizes and time lengths
%
% See Section III.B of the following paper for details
%   Title : Leading Cruise Control in Mixed Traffic Flow:
%                      System Modeling,Controllability,and String Stability
%   Author:Jiawei Wang, Yang Zheng, Chaoyi Chen, Qing Xu and Keqiang Li
% =========================================================================

clc;clear; close all;
addpath('..\_model');

% -------------------------------------------------------------------------
%   Parameter setup
% -------------------------------------------------------------------------

FD_bool = 1;            % mode of the LCC system
                        % 0. CF-LCC; 1. FD-LCC
s_star = 20;            % Equilibrium spacing
                        % correspond to an equilibrium velocity of 15 m/s

% ------------------------------------------
% Parameters in the car-following model
% ------------------------------------------
% Driver Model: OVM
alpha = 0.6;            
beta = 0.9;
s_st = 5;
s_go = 35;
v_max = 30;

% linearized model
alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
alpha2 = alpha+beta;
alpha3 = beta;
        
% ------------------------------------------
% Parameters for calculating the energy
% ------------------------------------------
T_collected = [10,20,30];   % Time length: t
N_collected = 1:5;          % System size: n

% -------------------------------------------------------------------------
% Energy-related metrics
% -------------------------------------------------------------------------
% Metric 1: smallest eigenvalue of Controllability Gramian: lambda_min(W(t))
MinEnergy = zeros(length(N_collected),length(T_collected));
% Metric 2: trace of inverse Controllability Gramian: Tr(W(t)^(-1))
AvgEnergy = zeros(length(N_collected),length(T_collected));
% Metric 3: minimum transfer energy: E_min(t)
TransferEnergy = zeros(length(N_collected),length(T_collected));

% -------------------------------------------------------------------------
% Some output information
% -------------------------------------------------------------------------
fprintf('=====================================================================\n')
fprintf('           Control Energy of Free-Driving LCC \n')
fprintf('                 By Jiawei Wang, Yang Zheng \n')
fprintf('=====================================================================\n')
fprintf('System size (number of HDVs behind) : ');fprintf('%d ',N_collected);fprintf('\n')
fprintf('Time length                         : ');fprintf('%d ',T_collected);fprintf('\n')
fprintf('HDV car-following model: optimal velocity model (OVM) \n')
fprintf('Parameter setup in HDV car-following model: \n')
fprintf('    alpha  beta  s_st  s_go  v_max \n    %4.2f  %4.2f  %4.2f  %4.2f  %4.2f\n',alpha,beta,s_st,s_go,v_max)
fprintf('Coefficients in linearized HDV car-following model: \n')
fprintf('    alpha1  alpha2  alpha3 \n    %4.2f    %4.2f    %4.2f \n',alpha1,alpha2,alpha3)
fprintf('-----------------------------------------------------------\n')
fprintf('   Numerical calculation beigns ...')

% -------------------------------------------------------------------------
% Numerical calculation starts here
% -------------------------------------------------------------------------
tic
h=waitbar(0,'please wait');
iTest = 0;
for iN = 1:length(N_collected)
    for iT = 1:length(T_collected)
        
        iTest = iTest+1;
        N = N_collected(iN);
        time = T_collected(iT);
      
        if FD_bool
            [A,B] = SystemModel_FD(N,alpha1,alpha2,alpha3);
        else
            [A,B] = SystemModel_CF(N,alpha1,alpha2,alpha3);
        end
                
        % Controllability Gramian
        % If the system is asymptotically stable, then Wc = lyap(A,B*B');
        % Here, we use the definition to calculate Wc.
        Wc = 0;        
        tau = 0.1;
        for x=0:tau:time
            Wc = Wc + expm(A*x)*(B*B')*expm(A'*x)*tau;
        end
                
        
        MinEnergy(iN,iT) = min(eig(Wc));
        
        AvgEnergy(iN,iT) = trace(inv(Wc));
        
        v = 1;
        % Equilibrium spacing with one-unit higher equilibrium velocity
        s = (alpha2-alpha3)/alpha1*v; 
        x = repmat([s,v]',N+1,1); % Target state
        TransferEnergy(iN,iT) = x'*Wc^(-1)*x;                
        
        str=['Processing...',num2str(iTest/length(N_collected)/length(T_collected)*100),'%'];
        waitbar(iTest/length(N_collected)/length(T_collected),h,str);
                
    end
end
close(h);
tsim = toc;

fprintf('  ends at %6.4f seconds \n', tsim);

% -------------------------------------------------------------------------
%  Plot Results 
% -------------------------------------------------------------------------
fprintf('-----------------------------------------------------------\n')
fprintf('    Now plot the results for control energy, please wait ... \n')

Wsize = 22;  % word size
Lwidth = 1.5;
Msize = 8;
color1 = [215 51 201]/255;
color2 = [242 60 84]/255;
color3 = [67, 121, 227]/255;

figure(1);
semilogy(N_collected,MinEnergy(:,1),'Color',color1,'linewidth',Lwidth); hold on;
p1 = semilogy(N_collected,MinEnergy(:,1),'o','Color',color1,'markersize',Msize,'linewidth',Lwidth);
semilogy(N_collected,MinEnergy(:,2),'Color',color2,'linewidth',Lwidth); hold on;
p2 = semilogy(N_collected,MinEnergy(:,2),'^','Color',color2,'markersize',Msize,'linewidth',Lwidth);
semilogy(N_collected,MinEnergy(:,3),'Color',color3,'linewidth',Lwidth); hold on;
p3 = semilogy(N_collected,MinEnergy(:,3),'x','Color',color3,'markersize',Msize,'linewidth',Lwidth);

grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',Wsize-5);
l = legend([p1,p2,p3],...
    ['$t=\,$',num2str(T_collected(1))],['$t=\,$',num2str(T_collected(2))],['$t=\,$',num2str(T_collected(3))],...
    'Location','NorthEast');
set(l,'fontsize',Wsize,'box','off','Interpreter','latex');

xlabel('$n$','fontsize',Wsize,'Interpreter','latex','Color','k');
title('$\lambda_{\mathrm{min}}(W(t))$','fontsize',Wsize,'Interpreter','latex','Color','k');

set(gcf,'Position',[250 150 300 450]);
set(gca,'YLim',[1e-15,10]);
set(gca,'XTick',0:2:6);
fig = gcf;
fig.PaperPositionMode = 'auto';


figure(2);
semilogy(N_collected,AvgEnergy(:,1),'Color',color1,'linewidth',Lwidth); hold on;
p1 = semilogy(N_collected,AvgEnergy(:,1),'o','Color',color1,'markersize',Msize,'linewidth',Lwidth);
semilogy(N_collected,AvgEnergy(:,2),'Color',color2,'linewidth',Lwidth); hold on;
p2 = semilogy(N_collected,AvgEnergy(:,2),'^','Color',color2,'markersize',Msize,'linewidth',Lwidth);
semilogy(N_collected,AvgEnergy(:,3),'Color',color3,'linewidth',Lwidth); hold on;
p3 = semilogy(N_collected,AvgEnergy(:,3),'x','Color',color3,'markersize',Msize,'linewidth',Lwidth);

grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',Wsize-5);
l = legend([p1,p2,p3],...
    ['$t=\,$',num2str(T_collected(1))],['$t=\,$',num2str(T_collected(2))],['$t=\,$',num2str(T_collected(3))],...
    'Location','NorthWest');
set(l,'fontsize',Wsize,'box','off','Interpreter','latex');

xlabel('$n$','fontsize',Wsize,'Interpreter','latex','Color','k');
title('$\mathrm{Tr}(W(t)^{-1} )$','fontsize',Wsize,'Interpreter','latex','Color','k');

set(gcf,'Position',[650 150 300 450]);
set(gca,'XTick',0:2:6);
fig = gcf;
fig.PaperPositionMode = 'auto';



figure(3);
semilogy(N_collected,TransferEnergy(:,1),'Color',color1,'linewidth',Lwidth); hold on;
p1 = semilogy(N_collected,TransferEnergy(:,1),'o','Color',color1,'markersize',Msize,'linewidth',Lwidth);
semilogy(N_collected,TransferEnergy(:,2),'Color',color2,'linewidth',Lwidth); hold on;
p2 = semilogy(N_collected,TransferEnergy(:,2),'^','Color',color2,'markersize',Msize,'linewidth',Lwidth);
semilogy(N_collected,TransferEnergy(:,3),'Color',color3,'linewidth',Lwidth); hold on;
p3 = semilogy(N_collected,TransferEnergy(:,3),'x','Color',color3,'markersize',Msize,'linewidth',Lwidth);

grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',Wsize-5);
l = legend([p1,p2,p3],...
    ['$t=\,$',num2str(T_collected(1))],['$t=\,$',num2str(T_collected(2))],['$t=\,$',num2str(T_collected(3))],...
    'Location','NorthWest');
set(l,'fontsize',Wsize,'box','off','Interpreter','latex');

xlabel('$n$','fontsize',Wsize,'Interpreter','latex','Color','k');
title('$E_{\mathrm{min}}(t)$','fontsize',Wsize,'Interpreter','latex','Color','k');

set(gcf,'Position',[1050 150 300 450]);
set(gca,'YLim',[1e-2,1e9]);
set(gca,'XTick',0:2:6);
fig = gcf;
fig.PaperPositionMode = 'auto';


% figure(1);
% print(gcf,'..\Figures\AvgEnergy','-painters','-depsc2','-r300');
% figure(2);
% print(gcf,'..\Figures\MinEnergy','-painters','-depsc2','-r300');
% figure(3);
% print(gcf,'..\Figures\TransferEnergy','-painters','-depsc2','-r300');
