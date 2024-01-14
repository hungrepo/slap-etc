%% Video
vid=0;

v = VideoWriter('Video_3Tracker_2D_no_rotate');
v.FrameRate=5;
 open(v);

fig1=figure(1);
% fig2=figure(2);
%colors
 
AUV_COL= [0 0 0];
 

AUV_COL33= [0.8 0.8 0]; % yellow
AUV_COL22= [0.1 0.1 0]; % black
AUV_COL11= [0.9 0.2 0]; % red
 
% yaw1=90-yaw1;
% yaw2=90-yaw2;
% yaw3=90-yaw3;
 
% yaw2_22=90-yaw22*180/pi;
% yaw3_33=90-yaw33*180/pi;

% Com1_old=COM1.Data(1);
% Com2_old=COM2.Data(1);
% Com3_old=COM3.Data(1);
 set (fig1, 'Units', 'normalized', 'Position', [0,0,1,1]);
 
M=200;
Sigma1_0=P1_save(:,1: 4);
Sigma2_0=P2_save(:,1: 4);
Sigma3_0=P3_save(:,1: 4);

mu1=Tracker1.x_hat(1,:);
mu2=Tracker2.x_hat(1,:);
mu3=Tracker3.x_hat(1,:);
    

 
% Data_Point1_0 = mvnrnd(mu1',(Sigma1_0+Sigma1_0.')/2, M);
% Data_Point2_0 = mvnrnd(mu2',(Sigma2_0+Sigma2_0.')/2, M);
% Data_Point3_0 = mvnrnd(mu3',(Sigma3_0+Sigma3_0.')/2, M);
    

    q1_hat1_0=[mu1(2),mu1(1)];
    covxx1=Sigma1_0(1,1);
    covyy1=Sigma1_0(2,2);
    Sigma1_0(1,1)=covyy1;
    Sigma1_0(2,2)=covxx1; 
    
    q1_hat2_0=[mu2(2),mu2(1)];
    covxx2=Sigma2_0(1,1);
    covyy2=Sigma2_0(2,2);
    Sigma2_0(1,1)=covyy2;
    Sigma2_0(2,2)=covxx2; 
    
    q1_hat3_0=[mu3(2),mu3(1)];
    covxx3=Sigma3_0(1,1);
    covyy3=Sigma3_0(2,2);
    Sigma3_0(1,1)=covyy3;
    Sigma3_0(2,2)=covxx3; 
    
    
    SigmaA=zeros(3,3);
    SigmaA(1:2,1:2)=Sigma1_0(1:2,1:2);
    SigmaB=zeros(3,3);
    SigmaB(1:2,1:2)=Sigma2_0(1:2,1:2);
    SigmaC=zeros(3,3);
    SigmaC(1:2,1:2)=Sigma3_0(1:2,1:2);
%     hold off;
N1=1;
N2=5000;
k=1;
i=1; 
while i<length(Tracker1.pd(:,1))
   
%     if k>20
    hold off;

     figure(fig1);

    % path black
   if i>N2
    plot(Tracker1.pd(i-N2:i,2),Tracker1.pd(i-N2:i,1),'-.','LineWidth',1,'Color',AUV_COL11);
    hold on;

    plot(Tracker2.pd(i-N2:i,2),Tracker2.pd(i-N2:i,1),'-.','LineWidth',1,'Color',AUV_COL22);
    plot(Tracker3.pd(i-N2:i,2),Tracker3.pd(i-N2:i,1),'-.','LineWidth',1,'Color',AUV_COL33);
   else
    plot(Tracker1.pd(1:i,2),Tracker1.pd(1:i,1),'-.','LineWidth',1,'Color',AUV_COL11);
    hold on;
    plot(Tracker2.pd(1:i,2),Tracker2.pd(1:i,1),'-.','LineWidth',1,'Color',AUV_COL22);
    plot(Tracker3.pd(1:i,2),Tracker3.pd(1:i,1),'-.','LineWidth',1,'Color',AUV_COL33);
   end    

    mu1=Tracker1.x_hat(i,:);
    mu2=Tracker2.x_hat(i,:);
    mu3=Tracker3.x_hat(i,:);
    
    q1_hat1=[mu1(2), mu1(1)];
    q1_hat2=[mu2(2), mu2(1)];
    q1_hat3=[mu3(2), mu3(1)];
 
    
    
    
    plot(Tracker1.q_hat(1:i,2),Tracker1.q_hat(1:i,1),'*','LineWidth',0.2,'Color',AUV_COL11);
    plot(Tracker1.q_hat(1,2),Tracker1.q_hat(1,1),'*','MarkerSize',15,'LineWidth',3,'Color',AUV_COL11);

    plot(Tracker2.q_hat(1:i,2),Tracker2.q_hat(1:i,1),'*','LineWidth',0.2,'Color',AUV_COL22);
    plot(Tracker2.q_hat(1,2),Tracker2.q_hat(1,1),'*','MarkerSize',15,'LineWidth',3,'Color',AUV_COL22);

    plot(Tracker3.q_hat(1:i,2),Tracker3.q_hat(1:i,1),'*','LineWidth',0.2,'Color',AUV_COL33);
    plot(Tracker3.q_hat(1,2),Tracker3.q_hat(1,1),'*','MarkerSize',15,'LineWidth',3,'Color',AUV_COL33);

    
    Sigma1=P1_save(:,4*i-3: 4*i);
    Sigma2=P2_save(:,4*i-3: 4*i);
    Sigma3=P3_save(:,4*i-3: 4*i);
    
%     Data_Point1 = mvnrnd(mu1',(Sigma1+Sigma1.')/2, M);
%     Data_Point2 = mvnrnd(mu2',(Sigma2+Sigma2.')/2, M);
%     Data_Point3 = mvnrnd(mu3',(Sigma3+Sigma3.')/2, M);
    
%     plot(Data_Point1(:,2),Data_Point1(:,1),'o','Color', AUV_COL11);
%     plot(Data_Point2(:,2),Data_Point2(:,1),'o','Color', AUV_COL22);
%     plot(Data_Point3(:,2),Data_Point3(:,1),'o', 'Color',AUV_COL33);
    
    
%     plot(Data_Point1_0(:,2),Data_Point1_0(:,1),'o','Color', AUV_COL11);
%     plot(Data_Point2_0(:,2),Data_Point2_0(:,1),'o','Color', AUV_COL22);
%     plot(Data_Point3_0(:,2),Data_Point3_0(:,1),'o', 'Color',AUV_COL33);


    covxx1=Sigma1(1,1);
    covyy1=Sigma1(2,2);
    Sigma1(1,1)=covyy1;
    Sigma1(2,2)=covxx1;
    % plot range singnals 
    if (rem(i-1,20)==0)
        h10 = plot([Tracker1.p(i,2) Target.q(i,2)],[Tracker1.p(i,1) Target.q(i,1)],'m-.','LineWidth',2)  ; 
        h11 = plot([Tracker2.p(i,2) Target.q(i,2)],[Tracker2.p(i,1) Target.q(i,1)],'m-.','LineWidth',2)  ; 
        h12 = plot([Tracker3.p(i,2) Target.q(i,2)],[Tracker3.p(i,1) Target.q(i,1)],'m-.','LineWidth',2)  ; 
    end
    
    h1= plot_gaussian_ellipsoid([q1_hat1_0 0], SigmaA,2);
    set(h1,'FaceColor',AUV_COL11); 
    set(h1,'EdgeColor',AUV_COL11); 
    set(h1,'LineStyle','None'); 
    set(h1,'FaceAlpha',0.3);
% 
    h2= plot_gaussian_ellipsoid(q1_hat1, Sigma1(1:2,1:2),2);
    set(h2,'Color',AUV_COL11); 

    covxx2=Sigma2(1,1);
    covyy2=Sigma2(2,2);
    Sigma2(1,1)=covyy2;
    Sigma2(2,2)=covxx2;
    
    h3= plot_gaussian_ellipsoid([q1_hat2_0 0], SigmaB,2);
    set(h3,'FaceColor',AUV_COL22); 
    set(h3,'EdgeColor',AUV_COL22); 
    set(h3,'LineStyle','None'); 
    set(h3,'FaceAlpha',0.3);

    
    h4= plot_gaussian_ellipsoid(q1_hat2, Sigma2(1:2,1:2),2);
    set(h4,'Color',AUV_COL22); 
    
    
    covxx3=Sigma3(1,1);
    covyy3=Sigma3(2,2);
    Sigma3(1,1)=covyy3;
    Sigma3(2,2)=covxx3;
    
    h5= plot_gaussian_ellipsoid([q1_hat3_0 0], SigmaC,2);
    set(h5,'FaceColor',AUV_COL33); 
    set(h5,'EdgeColor',AUV_COL33); 
    set(h5,'LineStyle','None'); 
    set(h5,'FaceAlpha',0.3);

    h6= plot_gaussian_ellipsoid(q1_hat3, Sigma3(1:2,1:2),2);
    set(h6,'Color',AUV_COL33); 

    
    plot(Target.q(1:i,2),Target.q(1:i,1),'-','LineWidth',4,'Color', 'b');
    plot(Target.q(i,2),Target.q(i,1),'*','MarkerSize',20, 'LineWidth', 3, 'Color', 'b');

    

%     plot(Tracker2.pd2(1:i,2),Tracker2.pd2(1:i,1),'-','LineWidth',0.2,'Color',[10,10,0]/255);
%     plot(Tracker3.pd3(1:i,2),Tracker3.pd3(1:i,1),'-','LineWidth',0.2,'Color',[200,10,10]/255);
    
if i>N1   
    plot(Tracker1.p(i-N1:i,2),Tracker1.p(i-N1:i,1),'.-','LineWidth',2,'Color',AUV_COL11);
    plot(Tracker2.p(i-N1:i,2),Tracker2.p(i-N1:i,1),'.-','LineWidth',2,'Color',AUV_COL22);
    plot(Tracker3.p(i-N1:i,2),Tracker3.p(i-N1:i,1),'.-','LineWidth',2,'Color',AUV_COL33);
else
    plot(Tracker1.p(1:i,2),Tracker1.p(1:i,1),'.-','LineWidth',2,'Color',AUV_COL11);
    plot(Tracker2.p(1:i,2),Tracker2.p(1:i,1),'.-','LineWidth',2,'Color',AUV_COL22);
    plot(Tracker3.p(1:i,2),Tracker3.p(1:i,1),'.-','LineWidth',2,'Color',AUV_COL33);
end
   
%     plot(Tracker2.p2(1:i,2),Tracker2.p2(1:i,1),'-','LineWidth',0.2,'Color',[10,10,0]/255);
%     plot(Tracker3.p3(1:i,2),Tracker3.p3(1:i,1),'-','LineWidth',0.2,'Color',[200,10,10]/255);
%     hold on;
%     view(0,60)    
    % path myellow
   Scale=1;
    GTF_Simulink_PlotAUV([Tracker1.p(i,2),Tracker1.p(i,1),0], [0,0,90-yaw1(i)+180], Scale, 0,AUV_COL11,1);  

    GTF_Simulink_PlotAUV([Tracker2.p(i,2),Tracker2.p(i,1),0], [0,0,90-yaw2(i)+180], Scale, 0,AUV_COL22,1);  

    GTF_Simulink_PlotAUV([Tracker3.p(i,2),Tracker3.p(i,1),0], [0,0,90-yaw3(i)+180], Scale, 0,AUV_COL33,1);  
if i<=100
axis([-80 60 -60 80]);
end

%     hold on;
    
    
%         view(0,50)    


    
    
%     GTF_Simulink_PlotAUV([Tracker2.p2(i,2),Tracker2.p2(i,1),0], [0,0,yaw2_22(i)], Scale, 0,AUV_COL22,1);  
%     GTF_Simulink_PlotAUV([Tracker3.p3(i,2),Tracker3.p3(i,1),0], [0,0,yaw3_33(i)], Scale, 0,AUV_COL33,1);  

    Scale=.25*8;
    grid on; %axis equal;
%     axis([-15 15 -5 25 ])
   
    title('Animation of Cooperative Target Tracking and Pursuit')
    xlabel('Y[m]')
    ylabel('X[m]')
%     axis equal;
    if(vid==1)
        drawnow;
        currFrame = getframe(fig1);
        writeVideo(v, currFrame);
    else
        pause(0);
        drawnow;
    end
%     hold off;
    
%     figure(fig2)
%     subplot(3,1,1);hold on;
%     if COM1.Data(i)~=Com1_old
%         stem(COM1.Time(i),1,'Color',[0.8,0.9,0]);
%         title('transmistion signal on vehicle 1');
%         Com1_old=COM1.Data(i);
%     end
%     subplot(3,1,2);hold on;
%     if COM2.Data(i)~=Com2_old
%         stem(COM2.Time(i),1,'k')
%         title('transmistion signal on vehicle 2');
%         Com2_old=COM2.Data(i);
%     end
%     subplot(3,1,3);hold on;
%     if COM3.Data(i)~=Com3_old
%         stem(COM3.Time(i),1,'r')
%         title('transmistion signal on vehicle 3')
%         Com3_old=COM3.Data(i);
%     end 
i = i+50;

if i>=100
    i=i+100;
    
end

 %  i=i+round(k);
%  k=k+0.1;
%  if k>=100
%      k=100;
%  end
end

if(vid==1)
    close(v);
%     close(fig);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

