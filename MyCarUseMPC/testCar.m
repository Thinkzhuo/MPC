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
step = 0.01; %????;
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
    MPC_Controller.Np =10;  %????;
    MPC_Controller.Nc = 2;  %????;
    MPC_Controller.Nx = 3;  %????;
    MPC_Controller.Nu = 2;  %?????;
    
end




%=========================????==========================================
while(1)
    %?????????;
    %==================??????=========================================
    
    
    
    
    
    %=====================================================================
    v = 10;
    delta_f = 0.1*sin(MyCar.time);
    MyCar.OnInput(v,delta_f);
    if(MyCar.time >=10)
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