%?????????;
clc
clear ;
close all;
disp('Start to Simulation...');
%====??????====================
disp('Load Reference data...');
load_ref_data;
disp('End of Load Reference data...');
%??????;
MyCar = Car;
%==========================??????======================================
step = 0.1; %????;
%==========================???????====================================
t = 0; %??????;
x = 0;
y = 0;
phi = 0;
MyCar.OnInitialize(t,x,y,phi);
%==========================?????????=================================
Timer = [];
X_car = [];
Y_car = [];
PHI_car = [];
Delta_f_Car = [];
Velocity_Car = [];
%===========================??????====================================
ControlMethod = 1; %1:MPC??;
%==============MPC?????????=========================================
if (ControlMethod == 1)
    %????;
    MPC_Controller.Np =5;  %????;
    MPC_Controller.Nc = 3;  %????;
    MPC_Controller.Nx = 3;  %????;
    MPC_Controller.Nu = 2;  %?????;
    %===============??????,?????????????;
    Xref = zeros(MPC_Controller.Np,1);
    Yref = zeros(MPC_Controller.Np,1);
    PHIref = zeros(MPC_Controller.Np,1);
    Q = 100*eye(MPC_Controller.Np+1,MPC_Controller.Np+1);
    R = 100*eye(MPC_Controller.Np+1,MPC_Controller.Np+1);
    
end
%=========================????==========================================
while(1)
    %?????????;
    %==================??????=========================================
    %?????????;
    t_current = MyCar.time; %????Xref,Yref?;
    formatSpec = 'Current time is %d\n.';
    str = sprintf(formatSpec,t_current);
    disp(str);
    %step1:?????reference??;
    for i = 1:MPC_Controller.Np
        [cur_x_ref,cur_y_ref,cur_phi_ref,cur_velocity_ref,cur_delta_ref] = get_current_ref(t_current);
        Xref(i,1) = cur_x_ref;
        Yref(i,1) = cur_y_ref;
        PHIref(i,1) = cur_phi_ref;
        t_current = t_current + step;
    end
    %step2:???????:
    %?????;
  %  lb = [0;-2;0;-2;0;-2];
  %  ub = [10;2;10;2;10;2];
     lb = [];
     ub = [];
    A = [];   %?????;
    b = [];   %?????;
    Aeq = []; %????;
    beq = []; %????;
    options = optimset('Algorithm','active-set');
    %=================??????==========================================
    [xval,fval,exitflag] = fmincon(@(u)MyMpcCost(MyCar.time,step,u,MyCar.x,MyCar.y,MyCar.phi,MPC_Controller.Np,MPC_Controller.Nc,MPC_Controller.Nu,Xref,Yref,PHIref,Q,R),...
        zeros(MPC_Controller.Nu*MPC_Controller.Nc,1),...
        A,b,Aeq,beq,lb,ub,[],options);
    %=====================================================================
    v = xval(1);
    delta_f = xval(2);
    MyCar.OnInput(v,delta_f);
    if(MyCar.time >= 20)
        break;
    end
    MyCar.RK45(step);
    %==================????=============================================
    Timer(end+1) = MyCar.time;
    X_car(end+1) = MyCar.x;
    Y_car(end+1) = MyCar.y;
    PHI_car(end+1) = MyCar.phi;
    Delta_f_Car(end+1) = MyCar.delta_f;
    Velocity_Car(end+1) = MyCar.v;
end
%======================????=============================================
figure
plot(X_car,Y_car,'k-','LineWidth',2);
xlabel('X/m')
ylabel('Y/m')
grid on
legend('Car Trajectory')
axis equal

figure
plot(Timer,PHI_car,'k-','LineWidth',2);
xlabel('time/s');
ylabel('\psi');
legend('\psi');
grid on

figure
plot(Timer,Delta_f_Car,'k-','LineWidth',2);
xlabel('time/s');
ylabel('\delta_f');
legend('\delta_f');
grid on

figure
plot(Timer,Velocity_Car,'k-','LineWidth',2);
xlabel('time/s');
ylabel('velocity');
legend('velocity');
grid on
disp('End of Simulation')
%????,????;
save('SimulationData.mat','Timer','X_car','Y_car','PHI_car','Delta_f_Car','Velocity_Car')