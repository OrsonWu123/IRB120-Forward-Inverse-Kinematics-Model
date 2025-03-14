close all;
clear;
clc;

%modified为改进型，这里为标准型
%改进型顺序：alpha\a\theta\d
%标准型顺序：theta\d\a\alpha
%默认为弧度制

%机器人建模
L(1)=Link('revolute','d',290,'a',0,'alpha',-pi/2);
L(2)=Link('revolute','d',0,'a',270,'alpha',0,'offset',-pi/2);
L(3)=Link('revolute','d',0,'a',70,'alpha',-pi/2);
L(4)=Link('revolute','d',302,'a',0,'alpha',pi/2);
L(5)=Link('revolute','d',0,'a',0,'alpha',-pi/2);
L(6)=Link('revolute','d',72,'a',0,'alpha',0);

L(1).qlim=[-165,165]*pi/180;
L(2).qlim=[-110,110]*pi/180;
L(3).qlim=[-110,70]*pi/180;
L(4).qlim=[-160,160]*pi/180;
L(5).qlim=[-120,120]*pi/180;
L(6).qlim=[-400,400]*pi/180;

Six_dof=SerialLink(L,'name','ABB IRB120');
Six_dof.base=transl(0,0,0);
q=[0 0 0 0 0 0];

%% %选择模块
teach_demo=0;
plot_demo=0;
working_space_demo=0;
square_trojectories_planning=0;
circle_trojectories_planning=1;
moveing=0;

if teach_demo  +plot_demo  +working_space_demo  +square_trojectories_planning  +circle_trojectories_planning   +moveing~=1
    error('错误！请一次运行一个项目!')
end

%% %试教窗口
if teach_demo
    Six_dof.teach;
    Six_dof.plot(q,'view',[45,20]);
end

%% %简单显示
if plot_demo
    q=[0 0 0 0 0 0];%[-0.0000   -0.2435   -1.1153   -0.0000   -0.2120    0.0000]
    T=forward_kinematics(Six_dof,q);
    %不知为何，theta2逆运动学限定-95度以上

    v=[45,20];
    Six_dof.plot(q,'view',v);
end

%% %工作空间（正运动学）
if working_space_demo
    P=working_space(Six_dof);
    plot3(P(:,1),P(:,2),P(:,3),'b.','MarkerSize',1);
    Six_dof.plot(q,'view',[0,90]);
end

%% %轨迹规划（方块）
if square_trojectories_planning
    T1=transl(500,100,300)*troty(pi)*trotz(pi);
    T2=transl(500,-100,300)*troty(pi)*trotz(pi);
    T3=transl(300,-100,300)*troty(pi)*trotz(pi);
    T4=transl(300,100,300)*troty(pi)*trotz(pi);
    t=linspace(0,2,50);
    full_t=linspace(0,8,200);

    rpy1=tr2rpy(T1);
    rpy2=tr2rpy(T2);
    rpy3=tr2rpy(T3);
    rpy4=tr2rpy(T4);

    rpy_traj1=mytraj(rpy1,rpy2,t);
    rpy_traj2=mytraj(rpy2,rpy3,t);
    rpy_traj3=mytraj(rpy3,rpy4,t);
    rpy_traj4=mytraj(rpy4,rpy1,t);
    T_traj_rot1=rpy2tr(rpy_traj1);
    T_traj_rot2=rpy2tr(rpy_traj2);
    T_traj_rot3=rpy2tr(rpy_traj3);
    T_traj_rot4=rpy2tr(rpy_traj4);

    P1=transl(T1);
    P2=transl(T2);
    P3=transl(T3);
    P4=transl(T4);
    P_traj1=mytraj(P1',P2',t);
    P_traj2=mytraj(P2',P3',t);
    P_traj3=mytraj(P3',P4',t);
    P_traj4=mytraj(P4',P1',t);
    T_traj_transl1=transl(P_traj1);
    T_traj_transl2=transl(P_traj2);
    T_traj_transl3=transl(P_traj3);
    T_traj_transl4=transl(P_traj4);

    n=length(t);

    T_traj1=zeros(4,4,n);
    T_traj2=zeros(4,4,n);
    T_traj3=zeros(4,4,n);
    T_traj4=zeros(4,4,n);
    for i=1:n
        T_traj1(:,:,i)=T_traj_transl1(:,:,i)*T_traj_rot1(:,:,i);
    end
    for i=1:n
        T_traj2(:,:,i)=T_traj_transl2(:,:,i)*T_traj_rot2(:,:,i);
    end
    for i=1:n
        T_traj3(:,:,i)=T_traj_transl3(:,:,i)*T_traj_rot3(:,:,i);
    end
    for i=1:n
        T_traj4(:,:,i)=T_traj_transl4(:,:,i)*T_traj_rot4(:,:,i);
    end

    T_traj=zeros(4,4,200);
    for j=1:50
        T_traj(:,:,j)=T_traj1(:,:,j);
    end
    for j=1:50
        T_traj(:,:,50+j)=T_traj2(:,:,j);
    end
    for j=1:50
        T_traj(:,:,100+j)=T_traj3(:,:,j);
    end
    for j=1:50
        T_traj(:,:,150+j)=T_traj4(:,:,j);
    end

    q_traj=Six_dof.ikunc(T_traj);

    rate1=gradient(q_traj(:,1),full_t(:));
    rate2=gradient(q_traj(:,2),full_t(:));
    rate3=gradient(q_traj(:,3),full_t(:));
    rate4=gradient(q_traj(:,4),full_t(:));
    rate5=gradient(q_traj(:,5),full_t(:));
    rate6=gradient(q_traj(:,6),full_t(:));

    qdd1=gradient(rate1(:),full_t(:));
    qdd2=gradient(rate2(:),full_t(:));
    qdd3=gradient(rate3(:),full_t(:));
    qdd4=gradient(rate4(:),full_t(:));
    qdd5=gradient(rate5(:),full_t(:));
    qdd6=gradient(rate6(:),full_t(:));

    
%{
    %演示
    grid on;
    xlabel('X-axis');
    ylabel('Y-axis');
    zlabel('Z-axis');
    title('Square Trojectories Planning');
    Six_dof.plot(q_traj,'view',[45,20],'trail','r');%,'movie','Square Trojectories Planning.gif'


    hold on;
%}
    %{
    %角度位置
    hold on;
    plot(full_t,q_traj(:,1),'.-','LineWidth',1);
    plot(full_t,q_traj(:,2),'.-','LineWidth',1);
    plot(full_t,q_traj(:,3),'.-','LineWidth',1);
    plot(full_t,q_traj(:,4),'.-','LineWidth',1);
    plot(full_t,q_traj(:,5),'.-','LineWidth',1);
    plot(full_t,q_traj(:,6),'.-','LineWidth',1);
    grid on;
    legend('axis1','axis2','axis3','axis4','axis5','axis6');
    xlabel('time(s)');
    ylabel('theta(pi)');
    title('Angles of Square Trojectories Planning');
    %}
%{
    %角速度
    hold on;
    plot(full_t,rate1(:),'.-','LineWidth',1);
    plot(full_t,rate2(:),'.-','LineWidth',1);
    plot(full_t,rate3(:),'.-','LineWidth',1);
    plot(full_t,rate4(:),'.-','LineWidth',1);
    plot(full_t,rate5(:),'.-','LineWidth',1);
    plot(full_t,rate6(:),'.-','LineWidth',1);
    grid on;
    legend('rate1','rate2','rate3','rate4','rate5','rate6');
    xlabel('time(s)');
    ylabel('angular velocity（pi/s)');
    title('Angular Velocity of Square Trojectories Planning');
%}

    hold on;
    plot(full_t,qdd1(:),'.-','LineWidth',1);
    plot(full_t,qdd2(:),'.-','LineWidth',1);
    plot(full_t,qdd3(:),'.-','LineWidth',1);
    plot(full_t,qdd4(:),'.-','LineWidth',1);
    plot(full_t,qdd5(:),'.-','LineWidth',1);
    plot(full_t,qdd6(:),'.-','LineWidth',1);
    grid on;
    legend('axis1','axis2','axis3','axis4','axis5','axis6');
    xlabel('time(s)');
    ylabel('angular acceleration(pi/s^2)');
    title('Angular Acceleration of Square Trojectories Planning');


end

%% %轨迹规划（圆弧）
if circle_trojectories_planning

    circle=spatial_circle(100,400,50,400,'z');
    full_t=linspace(0,1,100);
    T=transl(circle);
    
    %{
    q_circle=Six_dof.ikunc(T);
    %}
    q_circle2=zeros(100,6);
    for i=1:100
        T(:,:,i)=T(:,:,i)*troty(pi)*trotz(pi);%
        q_circle2(i,:)=inverse_kinematic(Six_dof,T(:,:,i));
    end

    
    grid on;
    title('Circle Trojectories');
    xlabel('X-axis');
    ylabel('Y-axis');
    zlabel('Z-axis');
    Six_dof.plot(q_circle2,'view',[45,20],'trail','r');%,'movie','Circle Trojectories Planning.gif'
%}
%{
    grid on;
    hold on;
    title('Circle Trojectories');
    plot(full_t,q_circle2(:,1),'.-','LineWidth',1);
    plot(full_t,q_circle2(:,2),'.-','LineWidth',1);
    plot(full_t,q_circle2(:,3),'.-','LineWidth',1);
    plot(full_t,q_circle2(:,4),'.-','LineWidth',1);
    plot(full_t,q_circle2(:,5),'.-','LineWidth',1);
    plot(full_t,q_circle2(:,6),'.-','LineWidth',1);
    legend('joint1','joint2','joint3','joint4','joint5','joint6');
    xlabel('time(s)');
    ylabel('angular(pi)');
%}

    %plot3(circle(:,1),circle(:,2),circle(:,3), 'r-', 'LineWidth', 2);

end

%% %轨迹规划
if moveing
    t=linspace(0,2,1001);
    
    q1=[pi -pi 0 0 0 0];
    q_test=zeros(6,6);

    grid on;
    xlabel('X-axis');
    ylabel('Y-axis');
    zlabel('Z-axis');

    for i=1:6
        q_num=q_test;
        q_test(i)=q1(i);
        q_move=mytraj(q_num,q_test,t);
        Six_dof.plot(q_move,'view',[45,20],'trail','r');
    end

end
