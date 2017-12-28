function cost = MyMpcCost(t,step,u,x,y,phi,Np,Nc,Nu,Xref,Yref,PHIref,Q,R)
%the Description of Input data:
% t:current time tk;
% step :Control and Simulation Step;
% u : the optimization values  length (u) = Nu*Nc;
% x,y,phi : measured car data:x , y, phi,
% Np : Predictive period;
% Nc : Control Period;
% Xref: reference sequence x;
% Yref: reference sequence y;
% PHIref: reference sequence PHI;
% weight matrix Q & R;
cost = 0; %numerical cost value ;
% Define a Car to Calculate the Car;
CostCar = Car; %
%=========================================================================
%Define a Initial Matrix to Save Data;
%=========================================================================

X_predict  = zeros(Np,1);
Y_predict  = zeros(Np,1);
PHI_predict  = zeros(Np,1);

X_error = zeros(Np+1,1);
Y_error = zeros(Np+1,1);
PHI_error = zeros(Np+1,1);

v = zeros(Np,1);
delta_f = zeros(Np,1);
%==========================================================================
%Start to Calculate the Cost Value;
%==========================================================================
for i = 1:Np
    if   i ==1
        %   Initialization;
        CostCar.OnInitialize(t,x,y,phi);
        v(i,1) = u(1);
        delta_f(i,1) = u(2);
        CostCar.OnInput(v(i,1),delta_f(i,1));
        %System forward;
        CostCar.RK45(step);
        %Get the reference;
        X_predict(i,1) = CostCar.x;
        Y_predict(i,1) = CostCar.y;
        PHI_predict(i,1) = CostCar.phi;
    else
        if (i <= Nc )
            v(i,1) = u(Nu*(i-1)+1);
            delta_f(i,1) = u(Nu*(i-1)+2);
        else
            v(i,1) = u(end-1);
            delta_f(i,1) = u(end);
        end
        CostCar.OnInput(v(i,1),delta_f(i,1));
        %System forward;
        CostCar.RK45(step);
        %Get the reference;
        X_predict(i,1) = CostCar.x;
        Y_predict(i,1) = CostCar.y;
        PHI_predict(i,1) = CostCar.phi;
    end
    %==========================================================================
    % Calculate the error Value;
    %==========================================================================
    X_real = zeros(Np+1,1);
    Y_real = zeros(Np+1,1);
    X_real(1,1) = x;
    X_real(2:Np+1,1) = X_predict;
    
    Y_real(1,1) = y;
    Y_real(2:Np+1,1) = Y_predict;
    
    X_error(i,1) = X_real(i,1) - Xref(i,1);
    Y_error(i,1) = Y_real(i,1) - Yref(i,1);
    PHI_error(i,1) = PHI_predict(i,1) - PHIref(i,1);
end
%==========================================================================
%Numrical Cost Value;
%==========================================================================
cost = cost + Y_error'*R*Y_error + X_error'*Q*X_error;