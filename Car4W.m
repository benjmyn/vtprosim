
classdef Car4W
    properties
        % Scalar Properties
        m(1,1);
            % Car mass (with driver)
        W(1,4);
            % (1) = FL
            % (2) = FR 
            % (3) = RL
            % (4) = RR
        a(1,1);
        b(1,1);
            % a, b = Lengths from F/R axle to CG
        tf(1,1);
        tr(1,1);
            % tf, tr = F/R track
        h(1,1);
        zf(1,1);
        zr(1,1);
            %zf, zr = F/R roll center height
        KF(1,1);
        KR(1,1);
            % kf, kr = F/R roll stiffness
        B4(1,4);
            % (1) = Max G's
            % (2) = G's lost per N
            % (3) = ???
            % (4) = ???
        Cd(1,1);
        Cl(1,1);
        A(1,1);
        ax_scale(1,1);
        rpm;
        torque;
        gear_ratio;
        trans_ratio;
        shift_vel;
        primary_drive(1,1);
        final_drive(1,1);
        wheel_radius(1,4);

        % Functions
        pajecka_fcn;
        motor_fcn;
        drag_fcn;
        lift_fcn;
        N_fcn;
        FxB_fcn;
        FxT_fcn;
        Fy_fcn;
        
    end
    methods
        function [Car] = Car4W(car_json,powertrain_json,World)
            % Constructor Function
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
            Car.W = Car.transferWeight(0,0);

            % Define Pajecka Function
            Car.pajecka_fcn = @(N) World.grip_scale .* (Car.B4(1) - Car.B4(2).*N) .* N ...
                                   .*heaviside(Car.B4(1) - Car.B4(2).*N); % if < 0, = 0

            % Define aerodynamic forces
            Car.drag_fcn = @(v) 1/2 .* Car.Cd .* Car.A .* World.p .* v.^2;
            Car.lift_fcn = @(v) 1/2 .* Car.Cl .* Car.A .* World.p .* v.^2; 
            
            % Define driving forces
            Car.N_fcn = @(v) (Car.W + Car.lift_fcn(v)./4);
                        
        end
        function W = transferWeight(Car,ax,ay)
            W = Car.m .* 9.8 ...
                .* (... 
                0.5 .* [Car.b,Car.b,Car.a,Car.a] ./ (Car.a+Car.b)... 
                + ...
                ay ./ [Car.tf,-Car.tf,Car.tr,-Car.tr] ...
                .* (...
                [Car.KF,Car.KF,Car.KR,Car.KR] .* (Car.h - mean([Car.zf,Car.zr])) ./ (Car.KF+Car.KR) ...
                + ...
                [Car.b,Car.b,Car.a,Car.a] .* [Car.zf,Car.zf,Car.zr,Car.zr] ./ (Car.a+Car.b) ...
                ) + ...
                [-1,-1,1,1] .* ax .* Car.h ./ (Car.a + Car.b));
        end
        function ax = findAX(Car,v)
            % Hone in on ax
            ax = 0;
            err = 1;
            W = Car.W;
            while abs(err) > 0.001
                % Re-transfer wheel weights
                W = Car.transferWeight(ax,0.0);

                % Determine front and rear accelerations
                ax_f = sum(Car.pajecka_fcn(W(1:2)) .* Car.ax_scale) ./ sum(W(1:2));
                ax_r = sum(Car.pajecka_fcn(W(3:4)) .* Car.ax_scale) ./ sum(W(3:4));
                
                % Determine error in ax
                ax_temp = ax;
                ax = min(ax_f,ax_r);
                err = ax - ax_temp;
            end
        end
        function ay = findAY(Car,v)
            % Hone in on ay
            ay = 0;
            err = 1;
            W = Car.W;
            while abs(err) > 0.001
                W = transferWeight(Car,0,ay);
                ay_temp = ay;
                ay_f = sum(Car.pajecka_fcn(W(1:2))) ./ sum(W(1:2));
                ay_r = sum(Car.pajecka_fcn(W(3:4))) ./ sum(W(3:4));
                ay = min(ay_f,ay_r);
                err = ay - ay_temp;
            end
        end
    end
end