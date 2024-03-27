
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

Car(1) = CarPM('c1.json','p1.json',World);

%% Plot GGV
% v = 0:30;
% plot(v,Car(1).axT_fcn(v), v, Car(1).axB_fcn(v), v, Car(1).ay_fcn(v))

%% Utilize GGV
figure
load("accel.mat")
% SWP = linspace(0,100,100);
% KWP = [100,linspace(0.04,0.04,48),0.1,linspace(0.06,0.06,50)];

RWP = abs(1./KWP);
% RWP(1) = 0.01; % has to be > 0
RWP(~isfinite(RWP)) = 10000;

VWP = cornVel(Car(1),SWP,RWP);
plot(SWP,VWP,'k+')
xlim([-1,SWP(end)+1])
ylim([0,40])
hold on


disp("Walking points...")
Log = walk(Car(1),SWP,VWP,RWP);
disp("Done walking points")

plot(SWP,Log.MVWP,'g.-')
Car(1).laptime = lapTime(SWP,Log.MVWP);
fprintf("Car 1 traversed track in %3.3f seconds.\n",Car(1).laptime)
xlabel("Distance (m)")
ylabel("Velocity (m/s)")

% figure
% plot3(GY,GX,MVWP,'.')


%% Functions
function Log = walk(Car,SWP,VWP,RWP)
    % Initialize velocity vectors
    UWP = SWP.*0; 
    a = SWP.*0;
    Log.MVWP = VWP.*0 + realmax;
    Log.GX = Log.MVWP;
    Log.GY = Log.MVWP;
    Log.gear = Log.MVWP.*0;
    Log.RPM = Log.MVWP.*0;
    
    for n = 1:length(SWP)
        % Set current waypoint velocity to max cornering velocity
        UWP(n) = VWP(n); 
        if VWP(n) < Log.MVWP(n)
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

            Log.GX =   Log.GX  .* (Log.MVWP <= UWP)        +      a./9.8 .* (Log.MVWP > UWP);
            Log.GY =   Log.GY  .* (Log.MVWP <= UWP) + (UWP.^2./RWP)./9.8 .* (Log.MVWP > UWP);
            Log.MVWP = Log.MVWP .* (Log.MVWP <= UWP)       +         UWP .* (Log.MVWP > UWP);
            
            
            % Display point progress (optional)
            fprintf(n+"/"+length(SWP)+"\n")
        end
    end
    [~,Log.gear,Log.RPM] = Car.powertrainLookup(Log.MVWP);
end

function VWP = cornVel(Car,SWP,RWP)
    disp("Calculating cornering velocities")
    VWP = SWP.*0;
    for i = 1:length(SWP)
        err = @(v) v.*abs(v) ./ RWP(i) - Car.ay_fcn(v);
        VWP(i) = fzero(err,0);
    end
    disp("Done calculating cornering velocities")
end

function t = lapTime(SWP,VWP)
    VWP(1) = 1;
    t = trapz(SWP, 1./(VWP));
end
    