% =========================================================================
%         Frequency-domain Response of LCC for a Perturbation Ahead
%
% Numerically calculate the magnitude of the head-to-tail transfer function 
% of LCC at various excitation frequencies, i.e., |G(jw)|, at different 
% feedback cases 
%
% See Section V.A of the following paper for details
%   Title : Leading Cruise Control in Mixed Traffic Flow:
%                      System Modeling,Controllability,and String Stability
%   Author:Jiawei Wang, Yang Zheng, Chaoyi Chen, Qing Xu and Keqiang Li
% =========================================================================

clc;clear; close all;

% -------------------------------------------------------------------------
%   Parameter setup
% -------------------------------------------------------------------------

m = 2;          % the number of preceding HDVs ahead
n = 2;          % the number of following HDVs behind

s_star = 20;    % Equilibrium spacing
                % correspond to an equilibrium velocity of 15 m/s

% ------------------------------------------
% Parameters in the car-following model
% ------------------------------------------
% Driver Model: OVM
alpha = 0.6;
beta = 0.9;
v_max  = 30;
s_st   = 5;
s_go   = 35;

% linearized model
alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
alpha2 = alpha+beta;
alpha3 = beta;

% -------------------------------------------------------------------------
% Parameters for feedback gains
% -------------------------------------------------------------------------
% feedback gains corresponding to vehicle -2
id_ahead2 = -2;
mu_ahead2 = 1;
k_ahead2 = -1;
% feedback gains corresponding to vehicle -1
id_ahead1 = -1;
mu_ahead1 = 1;
k_ahead1 = -1;
% feedback gains corresponding to vehicle 1
id_behind1 = 1;
mu_behind1 = -1;
k_behind1 = -1;
% feedback gains corresponding to vehicle 2
id_behind2 = 2;
mu_behind2 = -1;
k_behind2 = -1;

% -------------------------------------------------------------------------
% Some output information
% -------------------------------------------------------------------------
fprintf('-----------------------------------------------------------\n')
fprintf('           Frequency-domain Response of LCC \n')
fprintf('               By Jiawei Wang, Yang Zheng \n')
fprintf('Number of HDVs behind: %d\n',n)
fprintf('Number of HDVs ahead : %d\n',m)
fprintf('HDV car-following model  : %4.2f  %4.2f\n',alpha,beta)  % this can be improved
fprintf('-----------------------------------------------------------\n')
fprintf('   Numerical calculation beigns ...')

% -------------------------------------------------------------------------
% Calculate the head-to-tail transfer function of LCC
% -------------------------------------------------------------------------
tic
% HDV-only case
HeadTail_Tf0 = @(w)(abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m+1)));

% Case A: consider the information of vehicle -2
HeadTail_Tf1 = @(w)(abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m))...
    *abs((alpha3*1i*w+alpha1...
    +(mu_ahead2*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_ahead2+k_ahead2*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(id_ahead2+1))...
    /(-w^2+alpha1+1i*w*alpha2)));

% Case B: consider the information of vehicles -2, -1
HeadTail_Tf2 = @(w)(abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m))...
    *abs((alpha3*1i*w+alpha1...
    +(mu_ahead2*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_ahead2+k_ahead2*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(id_ahead2+1)...
    +(mu_ahead1*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_ahead1+k_ahead1*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(id_ahead1+1))...
    /(-w^2+alpha1+1i*w*alpha2)));

% Case C: consider the information of vehicles -2, -1, 1
HeadTail_Tf3 = @(w)(abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m))...
    *abs((alpha3*1i*w+alpha1...
    +(mu_ahead2*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_ahead2+k_ahead2*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(id_ahead2+1)...
    +(mu_ahead1*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_ahead1+k_ahead1*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(id_ahead1+1))...
    /(-w^2+alpha1+1i*w*alpha2...
    -(mu_behind1*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_behind1+k_behind1*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^id_behind1)));

% Case D: consider the information of vehicles -2, -1, 1, 2
HeadTail_Tf4 = @(w)(abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m))...
    *abs((alpha3*1i*w+alpha1...
    +(mu_ahead2*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_ahead2+k_ahead2*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(id_ahead2+1)...
    +(mu_ahead1*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_ahead1+k_ahead1*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(id_ahead1+1))...
    /(-w^2+alpha1+1i*w*alpha2...
    -(mu_behind1*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_behind1+k_behind1*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^id_behind1...
    -(mu_behind2*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_behind2+k_behind2*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^id_behind2...
    )));
tsim = toc;

fprintf('  ends at %6.4f seconds \n', tsim);

% -------------------------------------------------------------------------
%  Plot Results 
% -------------------------------------------------------------------------
fprintf('-----------------------------------------------------------\n')
fprintf('    Now plot the results for frequency-domain response, please wait ... \n')

Wsize = 22;

figure;
p0 = fplot(HeadTail_Tf0,[1e-8,3]);hold on;
p1 = fplot(HeadTail_Tf1,[1e-8,3]);hold on;
p2 = fplot(HeadTail_Tf2,[1e-8,3]);hold on;
p3 = fplot(HeadTail_Tf3,[1e-8,3]);hold on;
p4 = fplot(HeadTail_Tf4,[1e-8,3]);hold on;
plot(linspace(0,3,10),ones(1,10),'k--');

set(gca,'TickLabelInterpreter','latex','fontsize',Wsize-4);
xlabel('$\omega$','fontsize',Wsize,'Interpreter','latex','Color','k');
ylabel('$\vert\Gamma (j\omega) \vert$','fontsize',Wsize,'Interpreter','latex','Color','k');

colormap = [244, 53, 124;
    255, 176, 0;
    67, 121, 227;
    131,51,236]/255;

p0.LineWidth = 2;
p1.LineWidth = 2;
p2.LineWidth = 2;
p3.LineWidth = 2;
p4.LineWidth = 2;

p0.Color = 'k';
p1.Color = colormap(1,:);
p2.Color = colormap(2,:);
p3.Color = colormap(3,:);
p4.Color = colormap(4,:);

l = legend([p0,p1,p2,p3,p4],'HDV only','Case A','Case B','Case C','Case D');
l.Position = [0.55,0.4,0.3,0.3];
set(l,'fontsize',Wsize-2,'box','off','Interpreter','latex');

grid on;

set(gca,'ylim',[0,1.2]);
set(gca,'xlim',[0,3]);
set(gcf,'Position',[250 150 600 350]);
fig = gcf;
fig.PaperPositionMode = 'auto';

% print(gcf,['../Figures/Fig9_FrequencyResponse'],'-depsc','-r300');
