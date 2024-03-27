
classdef CarPM
    properties
        % World Coordinate System -> Z Down, X Forward, Y Right

        % Scalar Properties
        laptime(1,1);
        m(1,1);
        W(1,1);
        Cd(1,1);
        Cl(1,1);
        A(1,1);
        B4(1,4); 
            % (1) = Max G's
            % (2) = G's lost per N
            % (3) = ???
            % (4) = ???
        ax_scale(1,1);
        rpm;
        torque;
        gear_ratio;
        trans_ratio;
        shift_vel;
        primary_drive(1,1);
        final_drive(1,1);
        wheel_radius(1,1);
        
        % Functions
        pajecka_fcn;
        motor_fcn;
        drag_fcn;
        lift_fcn;
        
        N_fcn;
        FxB_fcn;
        FxT_fcn;
        Fy_fcn;
        
        axB_fcn;
        axT_fcn;
        ay_fcn;

        % Classes/Structs
        Log;

    end
    methods
        function [Car] = CarPM(car_json,powertrain_json,World)
            % Read .json to Struct
            CJ = readstruct(car_json);
            PJ = readstruct(powertrain_json);

            % Upload Struct Data to Object
            for fn = fieldnames(CJ)' 
                try 
                    Car.(fn{1}) = CJ.(fn{1});
                catch
                    warning("Unable to assign car field """+fn{1}+"""");
                end
            end
            for fn = fieldnames(PJ)' 
                try 
                    Car.(fn{1}) = PJ.(fn{1});
                catch
                    warning("Unable to assign powertrain field """+fn{1}+"""");
                end
            end

            % Resolve leftover properties
            Car.W = Car.m .* 9.8;
            Car.trans_ratio = Car.primary_drive .* Car.final_drive .* Car.gear_ratio;
            Car.shift_vel = Car.rpm(end) .* (2*pi/60) ./ Car.trans_ratio .* Car.wheel_radius;

            % Define Pajecka Function
            Car.pajecka_fcn = @(N) World.grip_scale .* (Car.B4(1) - Car.B4(2).*N) .* N ...
                                   .*heaviside(Car.B4(1) - Car.B4(2).*N); % if < 0, = 0

            % Define aerodynamic forces
            Car.drag_fcn = @(v) 1/2 .* Car.Cd .* Car.A .* World.p .* v.^2;
            Car.lift_fcn = @(v) 1/2 .* Car.Cl .* Car.A .* World.p .* v.^2; 

            % Define driving forces
            Car.N_fcn = @(v) (Car.W + Car.lift_fcn(v));
            Car.FxB_fcn = @(v) 4 .* Car.ax_scale .* Car.pajecka_fcn(Car.N_fcn(v) ./ 4) + Car.drag_fcn(v);
            Car.FxT_fcn = @(v) min(4 .* Car.ax_scale .* Car.pajecka_fcn(Car.N_fcn(v) ./ 4),...
                                   Car.powertrainLookup(v)) - Car.drag_fcn(v);
            Car.Fy_fcn = @(v) 4 .* Car.pajecka_fcn(Car.N_fcn(v) ./ 4);

            % Define GGV accelerations (in g's)
            Car.axB_fcn = @(v) Car.FxB_fcn(v) ./ Car.m;
            Car.axT_fcn = @(v) Car.FxT_fcn(v) ./ Car.m;
            Car.ay_fcn = @(v) Car.Fy_fcn(v) ./ Car.m;
        end
        function [Fx,cur_gear,rpm] = powertrainLookup(Car,v)
            Fx = linspace(0,0,length(v));
            cur_gear = linspace(0,0,length(v));
            rpm = linspace(0,0,length(v));
            torque = linspace(0,0,length(v));
            for vi = 1:length(v)
                if v(vi) >= Car.shift_vel(end)
                    cur_gear(vi) = length(Car.gear_ratio);
                    rpm(vi) = Car.rpm(end);
                    % torque = Car.torque(end) .* Car.trans_ratio()
                    Fx(vi) = Car.drag_fcn(v(vi));
                else
                    cur_gear(vi) = interp1([0,Car.shift_vel],0:length(Car.gear_ratio),v(vi),"next");
                    rpm(vi) = v(vi) ./ (Car.wheel_radius) .* (2*pi/60)^-1 .* Car.trans_ratio(cur_gear(vi));
                    torque(vi) = interp1(Car.rpm,Car.torque,rpm(vi),'linear') .* Car.trans_ratio(cur_gear(vi));
                    Fx(vi) = torque(vi) ./ Car.wheel_radius;
                end
            end
        end
    end
end