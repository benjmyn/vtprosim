
classdef Car4W
    properties
        % Vehicle Functions
        pajecka_fcn;
        drag_fcn;
        lift_fcn;
        axB_fcn;
        axT_fcn;
        ay_fcn;
        L_fcn;
        motor_fcn;
        % General Vehicle Properties
        m(1,1);
        W(1,4);
        laptime(1,1);
        % Tire Properties
        B4(1,4);
        ax_scale(1,1);
        % Suspension Properties
        a(1,1);
        b(1,1);
        l(1,1);
        h(1,1);
        tf(1,1);
        tr(1,1);
        KF(1,1);
        KR(1,1);
        zf(1,1);
        zr(1,1);
        % Aerodynamic Properties
        Cl(1,1);
        Cd(1,1);
        A(1,1);
        p(1,1);
        CoPa(1,1);
        CoPb(1,1);
        CoPh(1,1);
        % Powertrain Properties
        rpm;
        torque;
        gear_ratio;
        trans_ratio;
        shift_vel;
        primary_drive(1,1);
        final_drive(1,1);
        wheel_radius(1,1);
    end
    methods
        function Car = Car4W(carJson,powerJson,World)
            % Load properties from JSON files
            CJ = readstruct(carJson);
            PJ = readstruct(powerJson);
            for fn = fieldnames(CJ)' 
                try 
                    Car.(fn{1}) = CJ.(fn{1});
                catch
                    % warning("Unable to assign car field """+fn{1}+"""");
                end
            end
            for fn = fieldnames(PJ)' 
                try 
                    Car.(fn{1}) = PJ.(fn{1});
                catch
                    % warning("Unable to assign powertrain field """+fn{1}+"""");
                end
            end

            % Fill in derived properties
            Car.l = Car.a + Car.b;
            Car.W = Car.m .* 4.9 .* [Car.b,Car.b,Car.a,Car.a] ./ (Car.l);
            
            % Define aerodynamic forces
            Car.drag_fcn = @(v) 1/2 .* Car.Cd .* Car.A .* World.p .* v.^2;
            Car.lift_fcn = @(v) 1/2 .* Car.Cl .* Car.A .* World.p .* v.^2; 
            
            % Misc definitions
            Car.pajecka_fcn = @(N) World.grip_scale .* (Car.B4(1) - Car.B4(2).*N) .* N ...
                                   .*heaviside(Car.B4(1) - Car.B4(2).*N)... % if < 0, = 0
                                   .*heaviside(N); % if < 0, = 0
            Car.L_fcn = @(v) Car.lift_fcn(v) .* 0.5 .* [Car.CoPb,Car.CoPb,Car.CoPa,Car.CoPa] ./ (Car.l) ...
                         + Car.drag_fcn(v) .* Car.CoPh ./ (Car.l);
            Car.trans_ratio = Car.primary_drive .* Car.final_drive .* Car.gear_ratio;
            Car.shift_vel = Car.rpm(end) .* (2*pi/60) ./ Car.trans_ratio .* Car.wheel_radius;

            % Calculate FxB,FxT,Fy
            Car.axB_fcn = @(v) Car.approxAXB(v);
            Car.axT_fcn = @(v) Car.approxAXT(v);
            Car.ay_fcn = @(v) Car.approxAY(v);
        end
        function ax = approxAXB(Car,v)
            ax = 0;
            err = 1;
            while abs(err) > 0.001
                Wt = Car.W + Car.m .* ([-1,-1,1,1] .* ax .* Car.h ./ (Car.l));
                N = Wt - Car.L_fcn(v);
                No = Car.W - Car.L_fcn(v);
                ax_drag = Car.drag_fcn(v) ./ Car.m;
                ax_max = -sum(No(3:4)) .* (Car.l/2) ./ (Car.h .* Car.m);
                ax_f = sum(-Car.pajecka_fcn(N(1:2)).*Car.ax_scale) ./ sum(Wt(1:2) ./ 9.8);
                ax_temp = ax;
                ax = max([(ax_f - ax_drag),ax_max]);
                err = ax - ax_temp;
            end
        end
        function ax = approxAXT(Car,v)
            ax = 0;
            err = 1;
            while abs(err) > 0.001
                Wt = Car.W + Car.m .* ([-1,-1,1,1] .* ax .* Car.h ./ (Car.l));
                N = Wt - Car.L_fcn(v);
                No = Car.W - Car.L_fcn(v);
                ax_drag = Car.drag_fcn(v) ./ Car.m;
                ax_max = sum(No(1:2)) .* (Car.l/2) ./ (Car.h .* Car.m);
                ax_r = sum(+Car.pajecka_fcn(N(3:4)).*Car.ax_scale) ./ sum(Wt ./ 9.8);
                ax_eng = Car.powertrainLookup(v) ./ Car.m;
                ax_temp = ax;
                ax = min([ax_r-ax_drag,ax_eng-ax_drag,ax_max]);
                err = ax - ax_temp;
            end
        end
        function ay = approxAY(Car,v)
            ay = 0;
            err = 1;
            while abs(err) >= 0.001
                Wt = Car.W + Car.m .* (ay ./ [Car.tf,-Car.tf,Car.tr,-Car.tr] .* ...
                             [Car.KF,Car.KF,Car.KR,Car.KR] .* (Car.h - mean([Car.zf,Car.zr])) ./ ...
                             (Car.KF + Car.KR) + ...
                             [Car.b,Car.b,Car.a,Car.a] .* [Car.zf,Car.zf,Car.zr,Car.zr] ./ (Car.a+Car.b));
                N = Wt - Car.L_fcn(v);
                No = Car.W - Car.L_fcn(v);
                % ay_max = sum(No(2:3)) .* mean([Car.tf,Car.tr]) ./ (Car.h .* Car.m);
                ay_f = sum(Car.pajecka_fcn(N(1:2))) ./ sum(Wt(1:2)./9.8);
                ay_r = sum(Car.pajecka_fcn(N(3:4))) ./ sum(Wt(3:4)./9.8);
                ay_temp = ay;
                ay = min([ay_f,ay_r]);
                % disp([v,ay,ay_f,ay_r])
                err = ay - ay_temp;
            end
        end
        function [Fx] = powertrainLookup(Car,v)
            if v >= Car.shift_vel(end)
                cur_gear = length(Car.gear_ratio);
                rpm = Car.rpm(end);
                Fx = Car.drag_fcn(v);
            else
                cur_gear = interp1([-1,Car.shift_vel],0:length(Car.gear_ratio),v,"next");
                rpm = v ./ (Car.wheel_radius) .* (2*pi/60)^-1 .* Car.trans_ratio(cur_gear);
                torque = interp1(Car.rpm,Car.torque,rpm,'linear','extrap') .* Car.trans_ratio(cur_gear);
                Fx = torque ./ Car.wheel_radius;
            end
        end
    end
end
