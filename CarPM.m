
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
        gearing;
        
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
        
    end
    methods
        function [Car] = CarPM(json,World)
            % Read .json to Struct
            Str = readstruct(json);
            
            % Convert Struct to Object
            for fn = fieldnames(Str)' 
                try 
                    Car.(fn{1}) = Str.(fn{1});
                catch
                    warning("Unable to assign field """+fn{1}+"""");
                end
            end

            % Define Pajecka Function
            Car.pajecka_fcn = @(N) World.grip_scale .* (Car.B4(1) - Car.B4(2).*N) .* N ...
                                   .*heaviside(Car.B4(1) - Car.B4(2).*N); % if < 0, = 0

            % Define aerodynamic forces
            Car.drag_fcn = @(v) 1/2 .* Car.Cd .* Car.A .* World.p .* v.^2;
            Car.lift_fcn = @(v) 1/2 .* Car.Cl .* Car.A .* World.p .* v.^2; 

            % Define driving forces
            Car.W = Car.m .* 9.8;
            Car.N_fcn = @(v) (Car.W + Car.lift_fcn(v));
            Car.FxB_fcn = @(v) 4 .* Car.ax_scale .* Car.pajecka_fcn(Car.N_fcn(v) ./ 4) + Car.drag_fcn(v);
            Car.FxT_fcn = @(v) 4 .* Car.ax_scale .* Car.pajecka_fcn(Car.N_fcn(v) ./ 4) - Car.drag_fcn(v);
            Car.Fy_fcn = @(v) 4 .* Car.pajecka_fcn(Car.N_fcn(v) ./ 4);

            % Define GGV accelerations (in g's)
            Car.axB_fcn = @(v) Car.FxB_fcn(v) ./ Car.m;
            Car.axT_fcn = @(v) Car.FxT_fcn(v) ./ Car.m;
            Car.ay_fcn = @(v) Car.Fy_fcn(v) ./ Car.m;
        end
    end
end