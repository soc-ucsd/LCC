%% Description
% Calculate the control energy of FD-LCC ...
% at different system sizes and time lengths
% Correspond to Fig. 5 in our paper.

clc;
close all;
clear;
addpath('..\_model');

%% Parameters
FreeDriving = 1;
%1. FreeDriving 0.CarFollowing
DriverDynamics = 1;
switch DriverDynamics
    case 1
        alpha = 0.6;
        beta = 0.9;
    case 2
        alpha = 0.4;
        beta = 0.6;
end
s_star = 20;

%%
T_collected = [10,20,30];
N_collected = 1:5;

AvgEnergy = zeros(length(N_collected),length(T_collected));
MinEnergy = zeros(length(N_collected),length(T_collected));
TransferEnergy = zeros(length(N_collected),length(T_collected));


h=waitbar(0,'please wait');

iTest = 1;



for iN = 1:length(N_collected)
    for iT = 1:length(T_collected)
        
        
        N = N_collected(iN);
        time = T_collected(iT);
        
        OVM_bool = 1;
        
        
        %Driver Model: OVM
        alpha = 0.6;
        beta = 0.9;
        s_st = 5;
        s_go = 35;
        v_max = 30;
        
        
        if OVM_bool
            alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
            alpha2 = alpha+beta;
            alpha3 = beta;
        else
            % Completely controllable: alpha1-alpha2*alpha3+alpha3^2 \neq 0
            alpha1 = 1;
            alpha2 = 3;
            alpha3 = 1;
        end
        %Equilibrium
        % s_star = acos(1-v_star/v_max*2)/pi*(s_go-s_st)+s_st;
        
        
        %% SystemModel and Analysis
        
        if FreeDriving
            [A,B] = SystemModel_FD(N,alpha1,alpha2,alpha3);
        else
            %     [A,B] = SystemModel_CF_allHDV(N,alpha1,alpha2,alpha3);
            [A,B] = SystemModel_CF(N,alpha1,alpha2,alpha3);
        end
        
        
        %% Controllability Grammian
        
        % Wc = lyap(A,B*B');
        Wc = 0;
        
        tau = 0.1;
        for x=0:tau:time
            Wc = Wc + expm(A*x)*(B*B')*expm(A'*x)*tau;
        end
        
        
        AvgEnergy(iN,iT) = trace(inv(Wc));
        MinEnergy(iN,iT) = min(eig(Wc));
        
        v = 1;
        s = (alpha2-alpha3)/alpha1*v;
        x = repmat([s,v]',N+1,1);
        TransferEnergy(iN,iT) = x'*Wc^(-1)*x;
        
        
        
        str=['Processing...',num2str(iTest/length(N_collected)/length(T_collected)*100),'%'];
        waitbar(iTest/length(N_collected)/length(T_collected),h,str);
        
        iTest = iTest+1;
    end
end


%% Plot Figures

Wsize = 22;  % word size
Lwidth = 1.5;
Msize = 8;
color1 = [215 51 201]/255;
color2 = [242 60 84]/255;
color3 = [67, 121, 227]/255;


figure(1);
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

set(gcf,'Position',[250 150 300 450]);
set(gca,'XTick',0:2:6);
fig = gcf;
fig.PaperPositionMode = 'auto';

figure(2);
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

set(gcf,'Position',[250 150 300 450]);
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
