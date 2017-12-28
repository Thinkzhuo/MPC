classdef Car <handle
    %????;
    properties (Access = public)
        %***********????**********
        %4?????;
        time;  % s (????);
        x;     % m (????x);
        y;     % m (????y);
        phi;   % rad (?????);
        %2???;
        v;       % m/s (??);
        delta_f; % rad (?????);
        
        %??????;
        l = 2.98;   % m (??); %???1,?????200s,?????????????????????????;
        
        %????????????????
        xx;
        dx;
    end
    methods
        function RK45(obj,step)  %????;
            %ode45 method;
            a=zeros(5,1);
            a(1)=step/2; a(2)=step/2; a(5)=step/2;
            a(3)=step; a(4)=step;
            xw=obj.xx; x1=obj.xx; x2=obj.xx;
            for j=1:4
                obj.xx=x2;
                obj.OnStateUpdate();
                obj.OnDx();
                for i=1:4 %4?????;
                    x2(i)=a(j)*obj.dx(i)+x1(i);
                    xw(i)=a(j+1)*obj.dx(i)/3+xw(i);
                end
            end
            obj.xx=xw;
            obj.OnStateUpdate();
        end
        function OnDx(obj)
            obj.dx(1) = 1; %?????;
            obj.dx(2) = cos(obj.phi)*obj.v;
            obj.dx(3) = sin(obj.phi)*obj.v;
            obj.dx(4) = tan(obj.delta_f)/obj.l*obj.v;
        end
        function OnStateUpdate(obj)
            obj.time = obj.xx(1);
            obj.x = obj.xx(2);
            obj.y = obj.xx(3);
            obj.phi = obj.xx(4);
        end
        function OnInitialize(obj,t,x,y,phi)
            obj.xx(1) = t;
            obj.xx(2) = x;
            obj.xx(3) = y;
            obj.xx(4) = phi;
            obj.OnStateUpdate();
        end
        function OnInput(obj,velocity,delta_f)
            obj.v = velocity;
            obj.delta_f = delta_f;
        end
        
        
    end
end

