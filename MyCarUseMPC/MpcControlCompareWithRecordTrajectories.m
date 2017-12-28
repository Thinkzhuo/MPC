%MPC?????????????????;
%??????;
clear 
close all;
load('mpc_ref_data.mat'); %original data;
load('SimulationData.mat') %Simulation data;
%================?????????===========================================
figure 
plot(X_ref_final,Y_ref_final,'k-','LineWidth',2)
hold on
plot(X_car,Y_car,'r--','LineWidth',2)
grid on
xlabel('X/m');
ylabel('Y/m');
legend('original','simulation');
%===================???????????????==============================
figure
plot(angle_time,angle_ref,'k-','LineWidth',2);
hold on
plot(Timer,Delta_f_Car,'r--','LineWidth',2);
hold on
xlabel('time/s');
ylabel('\delta_f');
legend('\delta_f-original','\delta_f-simulation');
grid on
%==================?????????????===================================
figure
plot(Time,velocity_ref,'k-','LineWidth',2);
hold on
plot(Timer,Velocity_Car,'r--','LineWidth',2);
xlabel('time/s');
ylabel('velocity');
legend('velocity-original','velocity-simulation');
grid on
