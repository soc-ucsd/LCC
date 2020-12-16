% =========================================================================
%           Illustration for Desired Velocity Function in OVM
%
% Numerically calculate the three energy-related metrics of the FD-LCC 
% system at different system sizes and time lengths
%
% See II.A of the following paper for details
%   Title : Leading Cruise Control in Mixed Traffic Flow:
%                      System Modeling,Controllability,and String Stability
%   Author:Jiawei Wang, Yang Zheng, Chaoyi Chen, Qing Xu and Keqiang Li
% =========================================================================

clc;clear; close all;

% -------------------------------------------------------------------------
%   Parameter setup in OVM model
% -------------------------------------------------------------------------
alpha = 0.6;
beta = 0.9;
s_st = 5;
s_go = 35;
v_max = 30;

s = 0:0.1:40;                                   % spacing
v = v_max/2*(1-cos(pi*(s-s_st)/(s_go-s_st)));   % desired velocity
v(find(s<s_st)) = 0;
v(find(s>s_go)) = v_max;

% -------------------------------------------------------------------------
% Some output information
% -------------------------------------------------------------------------
fprintf('=====================================================================\n')
fprintf('      Illustration for Desired Velocity Function in OVM \n')
fprintf('                 By Jiawei Wang, Yang Zheng \n')
fprintf('=====================================================================\n')
fprintf('HDV car-following model: optimal velocity model (OVM) \n')
fprintf('Parameter setup in HDV car-following model: \n')
fprintf('    alpha  beta  s_st  s_go  v_max \n    %4.2f  %4.2f  %4.2f  %4.2f  %4.2f\n',alpha,beta,s_st,s_go,v_max)
fprintf('-----------------------------------------------------------\n')


% -------------------------------------------------------------------------
%  Plot Results 
% -------------------------------------------------------------------------
fprintf('    Now plot the illustration for desired velocity function, please wait ... \n')

f1 = figure(1);
plot(s,v,'linewidth',2);

hold on;
plot(s,v_max*ones(size(s)),'--k','linewidth',1);

Wsize = 14;  % word size
set(gca,'TickLabelInterpreter','latex','fontsize',11);
grid on;
xlabel('Spacing','fontsize',Wsize,'Interpreter','latex','Color','k');
ylabel('Desired velocity','fontsize',Wsize,'Interpreter','latex','Color','k');
axis([0 40 0 35]);

set(gca,'ytick',[])
set(gca,'xtick',[])

text(1,32,'$v_{\mathrm{max}}$','fontsize',Wsize,'Interpreter','latex','Color','k');

text(30,3,'$s_{\mathrm{go}}$','fontsize',Wsize,'Interpreter','latex','Color','k');
text(4.5,3,'$s_{\mathrm{st}}$','fontsize',Wsize,'Interpreter','latex','Color','k');
plot([5,5],[-1,1],'-k','linewidth',1);
plot([35,35],[0,30],'--k','linewidth',1);
set(gcf,'Position',[250 150 250 200]);

fig = gcf;
fig.PaperPositionMode = 'auto';
% print(gcf,'./Figures_Fig2_OVMSpacingPolicy','-painters','-depsc2','-r300');