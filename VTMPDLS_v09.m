
% SPONSORED BY RED BULL %

clc, clear, close all

% Define EACH car's characteristics from outside file (spreadsheet)

% Generate GGV for each car, store in Car.GGV

% Use GGV for each car to accelerate-from-point forwards & backwards 
% @ each point on track
% From there, choose minima of all points offered and get time (t) from vx(s)

% Start from rest in AutoX, Accel
% Measure best run in Endurance, Skidpad

% Store laptime data in Car.Laptime


%% Define obj characteristics
World.p = 1.225; % air density
World.grip_scale = 0.80; % tire grip factor vs. TTC data

load("accel.mat")
start_from_zero = true;

Paddock(1).car = Car4W('c4.json','p1.json',World);
Paddock(2).car = CarPM('c1.json','p1.json',World);

%% Plot GGV
% v = 0:30;
% plot(v,Car(1).axT_fcn(v), v, Car(1).axB_fcn(v), v, Car(1).ay_fcn(v))

%% Utilize GGV
RWP = abs(1./KWP);
if start_from_zero
    RWP(1) = 0.01; % has to be > 0
end
RWP(~isfinite(RWP)) = 10000;

for i = 1:length(Paddock)
    disp("Calculating cornering velocities for car "+i+" ...")
    VWP = cornVel(Paddock(i).car,SWP,RWP);
    disp("Done calculating cornering velocities for car "+i+" ...")
    
    Log(i).VWP = VWP;

    disp("Walking points for car "+i+" ...")
    [Log(i).MVWP,Log(i).GX,Log(i).GY] = walk(Paddock(i).car,SWP,VWP,RWP);
    disp("Done walking points for car "+i+" ...")
    
    Paddock(i).laptime = lapTime(SWP,Log(i).MVWP);
    Paddock(i).MVWP = Log.MVWP;
    fprintf("Car "+i+" completed track in %3.3f seconds.\n",Paddock(i).laptime);
    fprintf("\n");
end

% plot(SWP,Log(1).VWP,'b.',SWP,Log(2).VWP,'m.')
hold on
plot(SWP,Log(1).MVWP,'m-',SWP,Log(2).MVWP,'m--')
xlim([-1,SWP(end)+1])
ylim([0,40])
xlabel("Distance (m)")
ylabel("Velocity (m/s)")
legend("4-Wheel","Point Mass")
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
    