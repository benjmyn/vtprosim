
% (not) SPONSORED BY RED BULL %
clc, clear, close all

%% Define obj characteristics
World.p = 1.225; % air density
World.grip_scale = 0.80; % tire grip factor vs. TTC data

Track = load("moog.mat");
start_from_zero = true;

Paddock(1).car = Car4W('car_01.json','power_01.json',World);
Paddock(2).car = Car4W('car_02.json','power_02.json',World);
Paddock(1).car.final_drive = 3.00;
Paddock(1).car.update();

%% Utilize GGV
Track.RWP = abs(1./Track.KWP);
if start_from_zero
    Track.RWP(1) = 0.01; % has to be > 0
end
Track.RWP(~isfinite(Track.RWP)) = 10000;

for i = 1:length(Paddock)
    disp("Calculating cornering velocities for car "+i+" ...")
    VWP = cornVel(Paddock(i).car,Track.SWP,Track.RWP);
    disp("Done calculating cornering velocities for car "+i+" ...")
    
    Log(i).VWP = VWP;

    disp("Walking points for car "+i+" ...")
    [Log(i).MVWP,Log(i).GX,Log(i).GY] = walk(Paddock(i).car,Track.SWP,VWP,Track.RWP);
    disp("Done walking points for car "+i+" ...")
    
    Paddock(i).laptime = lapTime(Track.SWP,Log(i).MVWP);
    Paddock(i).MVWP = Log.MVWP;
    fprintf("Car "+i+" completed track in %3.3f seconds.\n",Paddock(i).laptime);
    fprintf("\n");
end

hold on
% plot(SWP,Log(1).VWP,'k.')
plot(Track.SWP,Log(1).MVWP,'m-', Track.SWP,Log(2).MVWP,'m--')
xlim([-1,Track.SWP(end)+1])
ylim([0,40])
xlabel("Distance (m)")
ylabel("Velocity (m/s)")
legend("WR-450","MT-07")
fprintf("Done!\n")

%% Functions
function [MVWP,GX,GY] = walk(Car,SWP,VWP,RWP)
    % Initialize velocity vectors
    UWP = SWP.*0; 
    a = SWP.*0;
    MVWP = VWP.*0 + realmax;
    GX = MVWP;
    GY = MVWP;
    gear = MVWP.*0;
    RPM = MVWP.*0;
    
    for n = 1:length(SWP)
        % Set current waypoint velocity to max cornering velocity
        UWP(n) = VWP(n); 
        if VWP(n) < MVWP(n)
            % Walk forward from n
            for i = (n+1):length(SWP)
                dx = SWP(i) - SWP(i-1); % cur-last
                a(i) = real(sqrt(Car.axT_fcn(UWP(i-1)).^2 - (Car.ax_scale.*UWP(i-1).^2./RWP(i)).^2));
                UWP(i) = sqrt(UWP(i-1).^2 + 2.*a(i).*dx);
            end
            % Walk backward from n
            for i = 1:(n-1)
                j = n-i; % reverse i
                dx = SWP(j+1) - SWP(j); % next-cur
                a(j) = -real(sqrt(Car.axB_fcn(UWP(j+1)).^2 - (Car.ax_scale.*UWP(j+1).^2./RWP(j)).^2));
                UWP(j) = sqrt(UWP(j+1).^2 - 2.*a(j).*dx);
            end

            GX =   GX  .* (MVWP <= UWP)        +      a./9.8 .* (MVWP > UWP);
            GY =   GY  .* (MVWP <= UWP) + (UWP.^2./RWP)./9.8 .* (MVWP > UWP);
            MVWP = MVWP .* (MVWP <= UWP)       +         UWP .* (MVWP > UWP);
        end
        % Display point progress (optional)
        if ~(mod(n,100))
            fprintf(n+"/"+length(SWP)+"\n")
        end
    end
end

function VWP = cornVel(Car,SWP,RWP)
    VWP = SWP.*0;
    for i = 1:length(SWP)
        err = @(v) v.*abs(v) ./ RWP(i) - Car.ay_fcn(v);
        VWP(i) = fzero(err,0);
    end
end

function t = lapTime(SWP,VWP)
    VWP(1) = 1;
    t = trapz(SWP, 1./(VWP));
end
    