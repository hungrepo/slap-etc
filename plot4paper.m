%  Plot data for paper presentation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot trajectory
fig2=figure(2);
set(fig2,'position',[0 0 550 350]);
%% Vehicle 1


h1=plot(Tracker1.pd(:,2),Tracker1.pd(:,1),'--','LineWidth',1,'Color',AUV_COL11);                                    
hold on;
h2=plot(Tracker1.p(:,2),Tracker1.p(:,1),'LineWidth',1,'Color',AUV_COL11);

h3=plot(Tracker2.pd(:,2),Tracker2.pd(:,1),'--','LineWidth',1,'Color',AUV_COL22);                                    
h4=plot(Tracker2.p(:,2),Tracker2.p(:,1),'LineWidth',1,'Color',AUV_COL22);

h5=plot(Tracker3.pd(:,2),Tracker3.pd(:,1),'--','LineWidth',1,'Color',AUV_COL33);                                    
h6=plot(Tracker3.p(:,2),Tracker3.p(:,1),'LineWidth',1,'Color',AUV_COL33);   


h7=plot(Tracker1.q_hat(:,2),Tracker1.q_hat(:,1),'*','MarkerSize',2,'LineWidth',0.2,'Color',AUV_COL11);
h8=plot(Tracker2.q_hat(:,2),Tracker2.q_hat(:,1),'*','MarkerSize',2,'LineWidth',0.2,'Color',AUV_COL22);
h9=plot(Tracker3.q_hat(:,2),Tracker3.q_hat(:,1),'*','MarkerSize',2,'LineWidth',0.2,'Color',AUV_COL33);

h10=plot(Target.q(:,2),Target.q(:,1),'-','LineWidth',2,'Color', 'b');



h20=plot(Tracker1.q_hat(1,2),Tracker1.q_hat(1,1),'*','MarkerSize',6,'LineWidth',1.5,'Color',AUV_COL11);
h21=plot(Tracker2.q_hat(1,2),Tracker2.q_hat(1,1),'*','MarkerSize',6,'LineWidth',1.5,'Color',AUV_COL22);
h22=plot(Tracker3.q_hat(1,2),Tracker3.q_hat(1,1),'*','MarkerSize',6,'LineWidth',1.5,'Color',AUV_COL33);


h30= plot_gaussian_ellipsoid([q1_hat1_0 0], SigmaA,2);
    set(h30,'FaceColor',AUV_COL11); 
    set(h30,'EdgeColor',AUV_COL11); 
    set(h30,'LineStyle','None'); 
    set(h30,'FaceAlpha',0.2);
 
    
    SigmaA=zeros(3,3);
    SigmaA(1:2,1:2)=Sigma1(1:2,1:2);
h31= plot_gaussian_ellipsoid([q1_hat1 0], SigmaA,2);
    set(h31,'FaceColor',AUV_COL11); 
    set(h31,'EdgeColor',AUV_COL11); 
    set(h31,'LineStyle','None'); 
    set(h31,'FaceAlpha',0.5);
    
    
h32= plot_gaussian_ellipsoid([q1_hat2_0 0], SigmaB,2);
    set(h32,'FaceColor',AUV_COL22); 
    set(h32,'EdgeColor',AUV_COL22); 
    set(h32,'LineStyle','None'); 
    set(h32,'FaceAlpha',0.2);
 
    
    SigmaB=zeros(3,3);
    SigmaB(1:2,1:2)=Sigma2(1:2,1:2);
h33= plot_gaussian_ellipsoid([q1_hat2 0], SigmaB,2);
    set(h33,'FaceColor',AUV_COL22); 
    set(h33,'EdgeColor',AUV_COL22); 
    set(h33,'LineStyle','None'); 
    set(h33,'FaceAlpha',0.5);    
    
    
h34= plot_gaussian_ellipsoid([q1_hat3_0 0], SigmaC,2);
    set(h34,'FaceColor',AUV_COL33); 
    set(h34,'EdgeColor',AUV_COL33); 
    set(h34,'LineStyle','None'); 
    set(h34,'FaceAlpha',0.2);
 
    
    SigmaC=zeros(3,3);
    SigmaC(1:2,1:2)=Sigma3(1:2,1:2);
h35= plot_gaussian_ellipsoid([q1_hat3 0], SigmaC,2);
    set(h35,'FaceColor',AUV_COL33); 
    set(h35,'EdgeColor',AUV_COL33); 
    set(h35,'LineStyle','None'); 
    set(h35,'FaceAlpha',0.5);     
    
    

    
%        plot(Data_Point1(:,2),Data_Point1(:,1),'o','Color', AUV_COL11);
grid on; %axis equal;
   
%     title('Animation of Coordinated PF')
    xlabel('Y[m]')
    ylabel('X[m]')
%      axis([-60 120 -60 60 ]);
GTF_Simulink_PlotAUV([Tracker1.p(end,2),Tracker1.p(end,1),0], [0,0,90-yaw1(end)+180], Scale*0.5 , 0,AUV_COL11,1);  
GTF_Simulink_PlotAUV([Tracker2.p(end,2),Tracker2.p(end,1),0], [0,0,90-yaw2(end)+180], Scale*0.5 , 0,AUV_COL22,1);  
GTF_Simulink_PlotAUV([Tracker3.p(end,2),Tracker3.p(end,1),0], [0,0,90-yaw3(end)+180], Scale*0.5 , 0,AUV_COL33,1);  


%lgd1=legend([h1 h2 h3 h4 h5 h6],'${\bf p}^{[1]}_{\rm d}$', '${\bf p}^{[1]}$','${\bf p}^{[2]}_{\rm d}$', '${\bf p}^{[2]}$','${\bf p}^{[3]}_{\rm d}$', '${\bf p}^{[3]}$');
lgd1=legend([h2 h4 h6],'${\bf p}^{[1]}$', '${\bf p}^{[2]}$', '${\bf p}^{[3]}$');

set(lgd1,'FontSize',12,'Interpreter','latex');
%lgd1.FontSize=11;

ah1=axes('position',get(gca,'position'),'visible','off');

lgd2=legend(ah1,[h7 h8 h9 h10],'$\hat{\bf q}^{[1]}$','$\hat{\bf q}^{[2]}$','$\hat{\bf q}^{[3]}$','${\bf q}$');
set(lgd2,'FontSize',12,'Interpreter','latex');
%lgd2.FontSize=11;

%% Plot error
L=length(time)-2500;
 
fig3=figure(3);
set(fig3,'position',[0 0 550 200]);
h40=plot(time(1:L),Tracker1.E_Pursuit(1:L),'LineWidth',1,'Color',AUV_COL11);
hold on;
h41=plot(time(1:L),Tracker2.E_Pursuit(1:L),'LineWidth',1,'Color',AUV_COL22);
h42=plot(time(1:L),Tracker3.E_Pursuit(1:L),'LineWidth',1,'Color',AUV_COL33);
 xlabel('$t(s)$','FontSize',12,'Interpreter','latex');
 ylabel('Errors[m]','FontSize',12,'Interpreter','latex');


h43=plot(time(1:L),Tracker1.E_Loc(1:L),'--','LineWidth',1,'Color',AUV_COL11);
h44=plot(time(1:L),Tracker2.E_Loc(1:L),'--','LineWidth',1,'Color',AUV_COL22);
h45=plot(time(1:L),Tracker3.E_Loc(1:L),'--','LineWidth',1,'Color',AUV_COL33);


lgd3=legend([h40 h41 h42],'$||{\bf{e}}^{[1]}||$ ', '$||{\bf{e}}^{[2]}||$', '$||{\bf{e}}^{[3]}||$');
set(lgd3,'FontSize',12,'Interpreter','latex');
%lgd3.FontSize=11;
ah1=axes('position',get(gca,'position'),'visible','off');
lgd4=legend(ah1, [h43 h44 h45], '$||\tilde{\bf{x}}^{[1]}||$', '$||\tilde{\bf{x}}^{[2]}||$', '$||\tilde{\bf{x}}^{[3]}||$' );
set(lgd4,'FontSize',12,'Interpreter','latex');
%lgd4.FontSize=11;

%% Plot evolution of path parameters
fig6=figure(6);
set(fig6,'position',[0 0 550 200]);
%n=length(time);
n=L;
plot(time(1:n),Tracker1.gamma(1:n),'LineWidth',1,'Color',AUV_COL11);
hold on;
plot(time(1:n),Tracker2.gamma(1:n),'LineWidth',1,'Color',AUV_COL22);
plot(time(1:n),Tracker3.gamma(1:n),'LineWidth',1,'Color',AUV_COL33);
xlabel('$t(s)$','FontSize',12,'Interpreter','latex');
ylabel('[rad]','FontSize',12,'Interpreter','latex');
lgd6=legend('${\gamma}^{[1]}$','${\gamma}^{[2]}$','${\gamma}^{[3]}$');
set(lgd6,'FontSize',12,'Interpreter','latex');
%% plot gamma_dot
fig7=figure(7);
set(fig7,'position',[0 0 550 150]);
%n=length(time);
n=L;
plot(time(1:n),Tracker1.gamma_dot(1:n),'LineWidth',1,'Color',AUV_COL11);
hold on;
plot(time(1:n),Tracker2.gamma_dot(1:n),'LineWidth',1,'Color',AUV_COL22);
plot(time(1:n),Tracker3.gamma_dot(1:n),'LineWidth',1,'Color',AUV_COL33);
xlabel('$t(s)$','FontSize',12,'Interpreter','latex');
ylabel('[rad/s]','FontSize',12,'Interpreter','latex');
plot(time(1:n), omega_bar*ones(1,n), 'b--','LineWidth',1 );
lgd7=legend('$\dot{\gamma}^{[1]}$','$\dot{\gamma}^{[2]}$','$\dot{\gamma}^{[3]}$','$\bar{\omega}$');
set(lgd7,'FontSize',12,'Interpreter','latex');

%set(fig7,'Interpreter','latex');



%% ========================== Plot for event triggered communications ==================================

% 
% %% Plot broadcast signals with cooperative pursuit 
% fontsize=12;
% 
%   fig10 = figure(10);
%   
%       set(fig10,'position',[0 0 550 200]);
%       idx = find(Tracker1.Com_Control(1,1:n)==1);
%       plot(time(idx),1*Tracker1.Com_Control(1,idx),'.','MarkerSize',8,'Color',AUV_COL11); hold on;
%       
%       idx = find(Tracker2.Com_Control(1,1:n)==1);
%       plot(time(idx),2*Tracker2.Com_Control(1,idx),'.','MarkerSize',8,'Color',AUV_COL22); hold on;
%       
%       idx = find(Tracker3.Com_Control(1,1:n)==1);
%       plot(time(idx),3*Tracker3.Com_Control(1,idx),'.','MarkerSize',8,'Color',AUV_COL33); hold on;
%     
% %   tmin=t(1); tmax=t(end)+1;
%    limit=[-0 200 -0 4];
%    axis(limit);
% %   set(gca,'YTick',[0 0.4 0.7 1]);
%   set(gca,'Yticklabel',[]); 
% %   grid on
%  % title('Broadcast time instants of the agents','Interpreter','latex');
%   xlabel('$t[s]$','Interpreter','latex','FontSize', fontsize);
%   ylabel('$t^{[i]}_n$','Interpreter','latex','FontSize', fontsize);
% 
% %  legend('Agent 1','Agent 2','Agent 3', 'Agent 4', 'Agent 5');
% 
% %% Plot time instant broadcast in DEKF
%  fig11 = figure(11);
%       m = 100;
%       set(fig11,'position',[0 0 550 200]);
%       idx = find(Tracker1.Com_DEKF(1,1:m)==1);
%       plot(time_d(idx),1*Tracker1.Com_DEKF(1,idx),'.','MarkerSize',8,'Color',AUV_COL11); hold on;
%       
%       idx = find(Tracker2.Com_DEKF(1,1:m)==1);
%       plot(time_d(idx),2*Tracker2.Com_DEKF(1,idx),'.','MarkerSize',8,'Color',AUV_COL22); hold on;
%       
%       idx = find(Tracker3.Com_DEKF(1,1:m)==1);
%       plot(time_d(idx),3*Tracker3.Com_DEKF(1,idx),'.','MarkerSize',8,'Color',AUV_COL33); hold on;
%     
% %   tmin=t(1); tmax=t(end)+1;
%    limit=[-0 100 -0 4];
%    axis(limit);
% %   set(gca,'YTick',[0 0.4 0.7 1]);
%   set(gca,'Yticklabel',[]); 
% %   grid on
%  % title('Broadcast time instants of the agents','Interpreter','latex');
%   xlabel('$k(t=kT_s)$','Interpreter','latex','FontSize', fontsize);
%   ylabel('$k^{[i]}_l$','Interpreter','latex','FontSize', fontsize);
% 
% %  legend('Agent 1','Agent 2','Agent 3', 'Agent 4', 'Agent 5');
% 
% 
%  
% %% Plot estimation error of gamma  
%   fig12 = figure(12);
%   set(fig12,'position',[0 0 550 400]);
% 
%       subplot(3,1,1);
%       plot(time(1:n),Tracker1.h(1,1:n),'LineWidth',1); hold on;
%       plot(time(1:n),abs(Tracker1.gamma_tilde(1,1:n)),'LineWidth',1,'Color',AUV_COL11); 
%       limit=[-0 200 -0 2.5];    axis(limit);
%       lg=legend('$h^{[1]}(t)$','$||{\tilde{\gamma}}^{[1]}(t)||$' );
%       set(lg,'Interpreter','latex');
%       
%       
%       subplot(3,1,2);
%       plot(time(1:n),Tracker1.h(1,1:n),'LineWidth',1); hold on;
%       plot(time(1:n),abs(Tracker2.gamma_tilde(1,1:n)),'LineWidth',1,'Color',AUV_COL22); 
%       limit=[-0 200 -0 2.5];    axis(limit);
%       lg=legend('$h^{[2]}$','$||{\tilde{\gamma}}^{[2]}(t)||$' );
%       set(lg,'Interpreter','latex');
%       
%       
%       subplot(3,1,3);
%       plot(time(1:n),Tracker3.h(1,1:n),'LineWidth',1); hold on;
%       plot(time(1:n),abs(Tracker3.gamma_tilde(1,1:n)),'LineWidth',1,'Color',AUV_COL33); 
%       limit=[-0 200 -0 2.5];    axis(limit);
%       lg=legend('$h^{[3]}(t)$','$||{\tilde{\gamma}}^{[3]}(t)||$' );
%       set(lg,'Interpreter','latex');
%       
%       
%  %% Plot estimation error KLD
%   fig13 = figure(13);
%   set(fig13,'position',[0 0 550 400]);
% 
%       subplot(3,1,1);
%       plot(time_d(1:m),Tracker1.g(1,1:m),'LineWidth',1); hold on;
%       plot(time_d(1:m),abs(Tracker1.KLD(1,1:m)),'LineWidth',1,'Color',AUV_COL11); 
%       limit=[-0 100 -0 20];    axis(limit);
%       lg=legend('$g^{[1]}(k)$','${\mathcal{KLD}}^{[1]}_{k}$' );
%       set(lg,'Interpreter','latex');
%       
%       
%       subplot(3,1,2);
%       plot(time_d(1:m),Tracker2.g(1,1:m),'LineWidth',1); hold on;
%       plot(time_d(1:m),abs(Tracker2.KLD(1,1:m)),'LineWidth',1,'Color',AUV_COL22); 
%       limit=[-0 100 -0 20];    axis(limit);
%       lg=legend('$g^{[2]}(k)$','${\mathcal{KLD}}^{[1]}_{k}$' );
%       set(lg,'Interpreter','latex');
%       
%       subplot(3,1,3);
%       plot(time_d(1:m),Tracker3.g(1,1:m),'LineWidth',1); hold on;
%       plot(time_d(1:m),Tracker3.KLD(1,1:m),'LineWidth',1,'Color',AUV_COL33); 
%       limit=[-0 100 -0 20];    axis(limit);
%       lg=legend('$g^{[3]}(k)$','${\mathcal{KLD}}^{[3]}_{k}$' );
%       set(lg,'Interpreter','latex');
%   
