%% Description
% Plot the new "looking ahead" head-to-tail string stable after incorporation of one vehicle behind 
% Correspond to Fig. 7(c)-(f) in our paper.

clc;
close all;
clear;


%% Key Parameters



% id of HDVs ahead
id_ahead = -1; % -1 or -2
% id of HDVs behind
id_behind = 1; % 1 or 2

%% Parameters

n = 2;
m = 2;

if id_behind == 1
    mu_behind = -1;
    k_behind = -1;
elseif id_behind == 2
    mu_behind = -1;
    k_behind = -1;
end

DriverDynamics = 1;

%% Load Data
load(['Experiment_Results\30-Mar-2020_SSRegion_n_',num2str(n),'_m_',num2str(m),...
    '_DriverDynamics_',num2str(DriverDynamics),'_FeedbackID_',num2str(id_ahead),'.mat']);
OriginalSS_bool = SS_bool;


if id_ahead<0
    load(['Experiment_Results\30-Mar-2020_SSRegion_WithLookingBehind_id_behind_',num2str(id_behind),...
        '_mu_behind_',num2str(mu_behind),'_k_behind_',num2str(k_behind)...
        '_n_',num2str(n),'_m_',num2str(m),...
        '_DriverDynamics_',num2str(DriverDynamics),'_FeedbackID_',num2str(id_ahead),'.mat']);
    FinalSS_bool = OriginalSS_bool+SS_bool;
    
    for i = 1:size(SS_bool,1)
        for j = 1:size(SS_bool,2)
            if OriginalSS_bool(i,j) == 1 && SS_bool(i,j) == 0
                FinalSS_bool(i,j) = 2;
            end
        end
    end
%% String Stability Area After Looking Behind
    

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


end