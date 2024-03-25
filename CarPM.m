
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
        function [Obj] = CarPM(json,World)
            % Read .json to Struct
            Str = readstruct(json);
            
            % Convert Struct to Object
            for fn = fieldnames(Str)' 
                try 
                    Obj.(fn{1}) = Str.(fn{1});
                catch
                    warning("Unable to assign field """+fn{1}+"""");
                end
            end

            % Define 4-term Pajecka Function
            % Obj.pajecka_fcn = @(a,N) (Obj.B4(1) - Obj.B4(2).*N).*N ...
            %                   .* sin(Obj.B4(4).*atan(Obj.B4(3).*a));
            Obj.pajecka_fcn = @(N) World.grip_scale .* (Obj.B4(1) - Obj.B4(2).*N) .* N ...
                                   .*heaviside(Obj.B4(1) - Obj.B4(2).*N); % if < 0, = 0

            % Define aerodynamic forces
            Obj.drag_fcn = @(v) 1/2 .* Obj.Cd .* Obj.A .* World.p .* v.^2;
            Obj.lift_fcn = @(v) 1/2 .* Obj.Cl .* Obj.A .* World.p .* v.^2; 

            % Define driving forces
            Obj.W = Obj.m .* 9.8;
            Obj.N_fcn = @(v) (Obj.W + Obj.lift_fcn(v));
            Obj.FxB_fcn = @(v) 4 .* Obj.ax_scale .* Obj.pajecka_fcn(Obj.N_fcn(v) ./ 4) + Obj.drag_fcn(v);
            Obj.FxT_fcn = @(v) 4 .* Obj.ax_scale .* Obj.pajecka_fcn(Obj.N_fcn(v) ./ 4) - Obj.drag_fcn(v);
            Obj.Fy_fcn = @(v) 4 .* Obj.pajecka_fcn(Obj.N_fcn(v) ./ 4);

            % Define GGV accelerations (in g's)
            Obj.axB_fcn = @(v) Obj.FxB_fcn(v) ./ Obj.m;
            Obj.axT_fcn = @(v) Obj.FxT_fcn(v) ./ Obj.m;
            Obj.ay_fcn = @(v) Obj.Fy_fcn(v) ./ Obj.m;
        end
    end
end