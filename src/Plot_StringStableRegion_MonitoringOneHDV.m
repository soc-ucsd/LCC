%% Description
% Plot the head-to-tail string stable region when monitoring one HDV
% Correspond to Fig. 7(a)(b) and Fig. 8(a)(b) in our paper.

clc;
close all;
clear;


%% Key Parameters
%%%%%%%%%%%%%%%%%%%%
% id of the HDV 
id = -2; % -2, -1, 1, 2
%%%%%%%%%%%%%%%%%%%%

%% Parameters

n = 2;
m = 2;


DriverDynamics = 1;


load(['Experiment_Results\18-Mar-2020_PSRegion_n_',num2str(n),'_m_',num2str(m),...
    '_DriverDynamics_',num2str(DriverDynamics),'_FeedbackID_',num2str(id),'.mat']);
SSPS_bool = zeros(size(PS_bool));
[Mu_3d,K_3d] = meshgrid(K,Mu);
a = find(PS_bool==0);
PS_Y = Mu_3d(a);
PS_X = K_3d(a);
PS_Z = 2*ones(length(a),1);

load(['Experiment_Results\30-Mar-2020_SSRegion_n_',num2str(n),'_m_',num2str(m),...
    '_DriverDynamics_',num2str(DriverDynamics),'_FeedbackID_',num2str(id),'.mat']);



%% Looking Behind
% Consider Plant Stability

figure;

mymap = [235,235,235;
    113, 178, 246]/255;
% Blue: original string stable region
% Red: expanded string stable region

colormap(mymap);

Wsize = 18;

[Mu_3d,K_3d] = meshgrid(K,Mu);

p = surf(K_3d,Mu_3d,SS_bool);

view([0 90]);

p.EdgeColor = 'none';

hold on;


set(gca,'TickLabelInterpreter','latex','fontsize',Wsize-2);
set(gca,'xlim',[min(K) max(K)]);
set(gca,'ylim',[min(Mu) max(Mu)]);

switch id
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

hold on;
s = scatter3(PS_X,PS_Y,PS_Z,2,'k','.');

if id >0
b = scatter3(-1,-1,2,50,'k','x');
b.LineWidth = 1.1;
text(-8,-1,2,'$(-1,-1)$','Interpreter','latex','fontsize',Wsize-2);
end

fig = gcf;
fig.PaperPositionMode = 'auto';

% print(gcf,['../Figures/Fig3_Driver_',num2str(DriverDynamics),'_id_',num2str(id)],'-depsc','-r300');

