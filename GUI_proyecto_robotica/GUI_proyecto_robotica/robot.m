clear;
clc;
close all;

dh = [
    0    0.089    .034       -pi/2;
    0    0        0.15   0;
    0    0        0.15   0
];


R = SerialLink(dh, 'name', 'Robot 3GDL');

R.offset = [0 -pi/2 0];

q = [0 0 0];


R.qlim(1,:) = [-180 180]*pi/180;
R.qlim(2,:) = [-90 90]*pi/180;
R.qlim(3,:) = [-90 90]*pi/180;

%% ============================================================
% VISUALIZACION
% =============================================================

figure;

R.plot(q, ...
    'workspace', [-0.3 0.3 -0.3 0.3 0 0.3], ...
    'scale', 0.6, ...
    'jointdiam', 0.4, ...
    'trail', {'r','LineWidth',2});

grid on;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Robot serie 3 GDL');
R.teach()