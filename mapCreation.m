
clc, clear, close all
T = readtable("test_03_30_24.csv");
XWP = T.x';
YWP = T.y';
KWP = getCurvature(XWP,YWP);
SWP = getDisplacement(XWP,YWP);

function [KWP] = getCurvature(XWP,YWP)
    X1 = XWP(1:end-2);
    X2 = XWP(2:end-1);
    X3 = XWP(3:end);
    Y1 = YWP(1:end-2);
    Y2 = YWP(2:end-1);
    Y3 = YWP(3:end);
    A = X1.*(Y2 - Y3) - Y1.*(X2 - X3) + X2.*Y3 - X3.*Y2;
    B = (X1.^2 + Y1.^2).*(Y3 - Y2) + (X2.^2 + Y2.^2).*(Y1 - Y3) + (X3.^2 + Y3.^2).*(Y2 - Y1);
    C = (X1.^2 + Y1.^2).*(X2 - X3) + (X2.^2 + Y2.^2).*(X3 - X1) + (X3.^2 + Y3.^2).*(X1 - X2);
    D = (X1.^2 + Y1.^2).*(X3.*Y2 - X2.*Y3) + (X2.^2 + Y2.^2).*(X1.*Y3 - X3.*Y1) + (X3.^2 + Y3.^2).*(X2.*Y1 - X1.*Y2);
    R = sqrt((B.^2 + C.^2 - 4.*A.*D) ./ (4.*A.^2));
    KWP = [1./R(1),1./R,1./R(end)];
end
function [SWP] = getDisplacement(XWP,YWP)
    X1 = XWP(1:end-1);
    X2 = XWP(2:end);
    Y1 = YWP(1:end-1);
    Y2 = YWP(2:end);
    dx = X2-X1;
    dy = Y2-Y1;
    ds = sqrt(dx.^2 + dy.^2);
    SWP = [0,cumsum(ds)];
end