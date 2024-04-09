
classdef Car4W < handle
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
        powertrainLookup;
        % General Vehicle Properties
        m(1,1);
        W(1,4);
        laptime(1,1);
        % Tire Properties
        P4(1,4);
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
            % Load property functions
            loadCarFcns(Car,World);
        end
    end
end
