
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

Car(1) = CarPM('c1.json',World);

%% Plot GGV
v = 0:30;
plot(v,Car(1).axT_fcn(v), v, Car(1).axB_fcn(v), v, Car(1).ay_fcn(v))

%% Utilize GGV
figure
load("VIR_clothoid.mat")
RWP = abs(1./KWP);
RWP(1) = 0.01; % has to be > 0
RWP(~isfinite(RWP)) = 10000;

VWP = cornVel(Car(1),SWP,RWP);
plot(SWP,VWP,'k>')
xlim([-1,SWP(end)+1])
ylim([0,40])
hold on

VWPMin = VWP.*0 + realmax; % Set VWPMin HIGH

disp("Walking points...")
for i = 1:length(VWP)
    WWP = walk(i,Car(1),SWP,VWP); % Walk from point
    VWPMin = VWPMin .* (VWPMin <= WWP) + WWP .* (VWPMin > WWP); 
    if ~(mod(i,100))
        fprintf("Passed point " + i + "\n");
    end
end
disp("Done walking points")

plot(SWP,VWPMin,'g.-')
Car(1).laptime = lapTime(SWP,VWPMin);
fprintf("Car 1 traversed track in %3.3f seconds.\n",Car(1).laptime)
xlabel("Distance (m)")
ylabel("Velocity (m/s)")


%% Functions
function UWP = walk(n,Car,SWP,VWP)
    UWP = SWP.*0;
    % Set current waypoint velocity to max cornering
    UWP(n) = VWP(n);
    % Walk forward from n
    for i = (n+1):length(SWP)
        dx = SWP(i) - SWP(i-1); % cur-last
        UWP(i) = sqrt(UWP(i-1).^2 + 0.5 .* Car.axT_fcn(UWP(i-1)) .* dx);
    end
    % Walk backward from n
    for i = 1:(n-1)
        j = n-i; % reverse i
        dx = SWP(j+1) - SWP(j); % next-cur
        UWP(j) = sqrt(UWP(j+1).^2 + 0.5 .* Car.axB_fcn(UWP(j+1)) .* dx);
    end
    
end

function VWP = cornVel(Car,SWP,RWP)
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
    