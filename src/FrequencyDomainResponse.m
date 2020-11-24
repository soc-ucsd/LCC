%% Description
% Plot the frequency-domain response of LCC systems
% Correspond to Fig. 9 in our paper.

clc;
clear;
close all;


%% 

m = 2;
n = 2;

s_star = 20;
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

%% 
id_ahead2 = -2;
mu_ahead2 = 1;
k_ahead2 = -1;

id_ahead1 = -1;
mu_ahead1 = 1;
k_ahead1 = -1;

id_behind1 = 1;
mu_behind1 = -1;
k_behind1 = -1;

id_behind2 = 2;
mu_behind2 = -1;
k_behind2 = -1;

%% 

HeadTail_Tf0 = @(w)(abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m+1)));
figure;
p0 = fplot(HeadTail_Tf0,[1e-8,3]);
hold on;



HeadTail_Tf1 = @(w)(abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m))...
    *abs((alpha3*1i*w+alpha1...
    +(mu_ahead2*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_ahead2+k_ahead2*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(id_ahead2+1))...
    /(-w^2+alpha1+1i*w*alpha2)));
p1 = fplot(HeadTail_Tf1,[1e-8,3]);
hold on;

HeadTail_Tf2 = @(w)(abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m))...
    *abs((alpha3*1i*w+alpha1...
    +(mu_ahead2*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_ahead2+k_ahead2*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(id_ahead2+1)...
    +(mu_ahead1*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_ahead1+k_ahead1*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(id_ahead1+1))...
    /(-w^2+alpha1+1i*w*alpha2)));
p2 = fplot(HeadTail_Tf2,[1e-8,3]);
hold on;



HeadTail_Tf3 = @(w)(abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m))...
    *abs((alpha3*1i*w+alpha1...
    +(mu_ahead1*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_ahead1+k_ahead1*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(id_ahead1+1))...
    /(-w^2+alpha1+1i*w*alpha2...
    -(mu_behind1*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_behind1+k_behind1*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^id_behind1)));

p3 = fplot(HeadTail_Tf3,[1e-8,3]);
hold on;

HeadTail_Tf4 = @(w)(abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m))...
    *abs((alpha3*1i*w+alpha1...
    +(mu_ahead1*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_ahead1+k_ahead1*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(id_ahead1+1))...
    /(-w^2+alpha1+1i*w*alpha2...
    -(mu_behind1*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_behind1+k_behind1*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^id_behind1...
    -(mu_behind2*((-w^2+alpha2*1i*w+alpha1)/(alpha3*1i*w+alpha1))-mu_behind2+k_behind2*1i*w)...
    *((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^id_behind2...
    )));

p4 = fplot(HeadTail_Tf4,[1e-8,3]);
hold on;

plot(linspace(0,3,10),ones(1,10),'k--');
%% 
Wsize = 22;

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
