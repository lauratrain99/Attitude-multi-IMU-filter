clear;clc;

load('rocketdata1.mat');

rocketdata1 = rocketdata.rocketdata1;

N = 84001;
M = 87719;

t = rocketdata1(N:M,1) - rocketdata1(N,1);
lat = rocketdata1(N:M,2);
lon = rocketdata1(N:M,3);
alt = rocketdata1(N:M,4);
v1 = rocketdata1(N:M,5);
v2 = rocketdata1(N:M,6);
v3 = rocketdata1(N:M,7);
a1 = rocketdata1(N:M,8);
a2 = rocketdata1(N:M,9);
a3 = rocketdata1(N:M,10);
j1 = rocketdata1(N:M,11);
j2 = rocketdata1(N:M,12);
j3 = rocketdata1(N:M,13);
e1 = rocketdata1(N:M,14);
e2 = rocketdata1(N:M,15);
e3 = unwrap(rocketdata1(N:M,16));

% compute angular velocity
w1 = zeros(1,length(t))';
w2 = zeros(1,length(t))';
w3 = zeros(1,length(t))';

for i = 2:length(t)
    dt = t(i) - t(i-1);
    w1(i) = (e1(i) - e1(i-1))/dt;
    w2(i) = (e2(i) - e2(i-1))/dt;
    w3(i) = (e3(i) - e3(i-1))/dt;
end

wv = [zeros(1,length(t))', w2, w3];

sim.wv = wv;
sim.t = t;
sim.ini_align = [0; 0; 0];
% [sim] = IMU_simulator(sim);

figure(1)
plot(t, v1, 'r', t, v2, 'b', t, v3, 'g', 'LineWidth',2)
title('velocity [m/s] vs time')
xlabel('t [s]')
ylabel('v [m/s]')
grid minor

figure(2)
plot(t, a2, 'b', t, a3, 'g', 'LineWidth',2)
title('acceleration [m/s^2] vs time')
xlabel('t [s]')
ylabel('a [m/s^2]')
grid minor

figure(3)
plot(t, rad2deg(e2), 'b', t, rad2deg(e3), 'g', 'LineWidth',2)
title('Euler angles [deg] vs time')
xlabel('t [s]')
ylabel('Euler angles [deg]')
legend('e2','e3')
grid minor

figure(4)
plot(t, alt, 'b', 'LineWidth',2)
title('altitude [m] vs time')
xlabel('t [s]')
ylabel('altitude [m]')
grid minor

figure(5)
worldmap('Morocco')
load coastlines
plotm(coastlat,coastlon, 'b', 'LineWidth',2)
hold on
plotm(rad2deg(lat),rad2deg(lon), 'r*')
hold off

figure(6)
plot(t, rad2deg(w2), 'b', t, rad2deg(w3), 'g', 'LineWidth',2)
title('angular velocity [rad/s] vs time')
legend('w2','w3')
xlabel('t [s]')
ylabel('w [rad/s]')
grid minor

% [quat, euler] = attitude_computer(sim);

figure(7)
plot(t, euler(:,1), 'r', t, euler(:,2), 'b', t, euler(:,3), 'g', 'LineWidth',2)
title('angular velocity [rad/s] vs time')
xlabel('t [s]')
ylabel('w [rad/s]')
grid minor
