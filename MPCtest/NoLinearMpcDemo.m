% 非线性MPC的主程序demo;
clc;
clear ;
close all;
%==========================================================================
%参数初始化;
%==========================================================================
Nx = 3; %状态量个数;
Np = 30; %预测时域;
Nc = 2; %控制时域；
l = 1; %车辆轴距;

State_Initial = zeros(Nx,1); %状态矩阵初始值;
State_Initial(1,1) = 0;  %x；
State_Initial(2,1) = 0;  %y;
State_Initial(3,1) = pi/6; %phi；

Q = 100*eye(Np+1,Np+1); %权重系数;
R = 100*eye(Np+1,Np+1); %权重系数;

%==========================================================================
%参考轨迹生成;
%==========================================================================
N =100 ; %参考轨迹点数量;
T = 0.05 ; %采样周期;
Xref = zeros(Np,1);
Yref = zeros(Np,1);
PHIref = zeros(Np,1);
%==========================================================================
%开始进行求解;
%==========================================================================
for j=1:1:N
    %=生成参考轨迹;
    for Nref =1:1:Np
        Xref(Nref,1) = (j+Nref-1)*T;
        Yref(Nref,1) = 2;
        PHIref(Nref,1) = 0;
    end
    lb = [0.8;-0.44;0.8;-0.44];
    ub = [1.2;0.44;1.2;0.44];
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    options = optimset('Algorithm','active-set');
    [A,fval,exitflag] = fmincon(@(x)MY_costfunction(x,State_Initial,...
        Np,Nc,T,Xref,Yref,PHIref,Q,R),[0;0;0;0],...
        A,b,Aeq,beq,lb,ub,[],options);%有约束求解;
    v_actual = A(1);
    deltaf_actual  = A(2);
    
    %车辆系统推进;
    X00(1) = State_Initial(1,1);
    X00(2) = State_Initial(2,1);
    X00(3) = State_Initial(3,1);
    XOUT = dsolve('Dx - v_actual*cos(z)=0','Dy-v_actual*sin(z)= 0','Dz-v_actual*tan(deltaf_actual)=0',...
        'x(0)=X00(1)','y(0)=X00(2)','z(0)=X00(3)');
    
    t = T;
    State_Initial(1,1) = eval(XOUT.x);
    State_Initial(2,1) = eval(XOUT.y);
    State_Initial(3,1) = eval(XOUT.z);
    figure(1)
    plot(State_Initial(1,1),State_Initial(2,1),'ko');
    hold on;
    plot([0,5],[2,2],'r-');
    hold on
    axis([0 5 0 4])
    
    
    
end