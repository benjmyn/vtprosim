
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
Environment.p = 1.225; % air density
Environment.grip_factor = 0.80; % tire grip factor vs. TTC data

Car(1).m = 167 + 90.7; % KG
Car(1).ax = 1.0.*9.8; % m/s^2
Car(1).ay = 1.5.*9.8; % m/s^2 
Car(1).Cd = 1.0; 
Car(1).Cl = -1.0;
Car(1).A = 0.90; % m^2
Car(1).TopSpeed = 35; % m/s
Car(1).GGV.drag = @(v) Car(1).Cd .* Car(1).A .* Environment.p .* v.^2 ./ 2;
Car(1).GGV.lift = @(v) Car(1).Cl .* Car(1).A .* Environment.p .* v.^2 ./ 2;
Car(1).GGV.axT = @(v) (Car(1).ax - Car(1).GGV.drag(v)./Car(1).m).*(v<=Car(1).TopSpeed); % positive, simplified
Car(1).GGV.axB = @(v) (Car(1).ax + Car(1).GGV.drag(v)./Car(1).m).*(v<=Car(1).TopSpeed); % positive, simplified
Car(1).GGV.ay = @(v) Car(1).ay + 0.1.*v; % positive, simplified

%% Plot GGV
% v = 0:30;
% plot(v,Car(1).GGV.axT(v), v, Car(1).GGV.axB(v), v, Car(1).GGV.ay(v))
% yline(0)

%% Utilize GGV
figure
% SWP = [0, 75, 100];
% RWP = [0, 61.2, 27.2];
% VWP = [0, 30, 20];

load("VIR_clothoid.mat")
RWP = abs(1./KWP);
RWP(1) = 0;
RWP(~isfinite(RWP)) = 10000;

VWP = cornVel(Car(1),SWP,RWP);
plot(SWP,VWP,'k>')
xlim([-1,SWP(end)+1])
ylim([0,40])
hold on

VWPMin = VWP.*0 + realmax; % Set VWPMin HIGH

for i = 1:length(VWP)
    WWP = walk(i,Car(1),SWP,VWP); % Walk from point
    VWPMin = VWPMin .* (VWPMin <= WWP) + WWP .* (VWPMin > WWP); 
    disp(i);
end
plot(SWP,VWPMin,'g.-')
Car(1).LapTime = lapTime(SWP,VWPMin);
disp(Car(1).LapTime)
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
        UWP(i) = sqrt(UWP(i-1).^2 + 0.5 .* Car.GGV.axT(UWP(i-1)) .* dx);
    end
    % Walk backward from n
    for i = 1:(n-1)
        j = n-i; % reverse i
        dx = SWP(j+1) - SWP(j); % next-cur
        UWP(j) = sqrt(UWP(j+1).^2 + 0.5.* Car.GGV.axT(UWP(j+1)) .* dx);
    end
end

function VWP = cornVel(Car,SWP,RWP)
    persistent iter
    VWP = SWP.*0;
    for i = 1:length(SWP)
        err = 1;
        iter = 0;
        while abs(err) > 0.01
            iter = iter + 1;
            VWP(i) = (Car.GGV.ay(VWP(i)) .* RWP(i)) .^ 0.5;
            err = VWP(i).^2 ./ RWP(i) - Car.GGV.ay(VWP(i)); % compare acceleration
            disp([iter, abs(err)])
        end
    end
    disp("Done calculating cornering velocities")
end

function t = lapTime(SWP,VWP)
    VWP(1) = 1;
    t = trapz(SWP, 1./(VWP));
end
    