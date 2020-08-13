clear;clc;
%% read sensor data
newData = importdata('202005100922.txt', '\t', 2);
sampleFrequency=100;
samplePeriod=1/sampleFrequency;

data=newData.data;
datalen=size(data,1);
acc=data(:,1:3); % (g)
gyro=data(:,4:6); % (deg/s)
oangles=data(:,7:9); % (deg)
%Temp=data(:,10);
mag=data(:,11:13);
oq=data(:,14:17);

%% prepare sensor data
time=(1:datalen)';time=time*samplePeriod;
Gyroscope=gyro;
Accelerometer=acc;
Magnetometer=mag;

%% plot sensor data
figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(time, Magnetometer(:,1), 'r');
plot(time, Magnetometer(:,2), 'g');
plot(time, Magnetometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
hold off;
linkaxes(axis, 'x');

%% Process sensor data through algorithm
initBeta=0.6;Beta=0.3;
initTime=5*sampleFrequency;

AHRS = MadgwickAHRS('SamplePeriod', samplePeriod, 'Beta', initBeta);
%AHRS = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 0.5);

quaternion = zeros(length(time), 4);

for t = 1:initTime
    AHRS.Update(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end
AHRS.Beta=Beta;
for t = initTime+1:length(time)
    AHRS.Update(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ?90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure('Name', 'Euler Angles');
hold on;
plot(time, euler(:,1), 'r');
plot(time, euler(:,2), 'g');
plot(time, JYcontinuous(euler(:,3)), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

%% plot JY901 angles as reference

figure('Name', 'JY901 Angles');
hold on;
plot(time, oangles(:,1), 'r');
plot(time, oangles(:,2), 'g');
plot(time, JYcontinuous(oangles(:,3)), 'b');
title('JY901 angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

%% plot algorithm output as quaternion

% figure('Name', 'quaternion');
% hold on;
% plot(time, quaternion(:,1), 'r');
% plot(time, quaternion(:,2), 'g');
% plot(time, quaternion(:,3), 'b');
% plot(time, quaternion(:,4), 'k');
% title('quaternion');
% xlabel('Time (s)');
% ylabel('quaternion');
% legend('1', '2', '3','4');
% hold off;

%% plot JY901 quaternion as reference

% figure('Name', 'quaternion');
% hold on;
% plot(time, oq(:,1), 'r');
% plot(time, oq(:,2), 'g');
% plot(time, oq(:,3), 'b');
% plot(time, oq(:,4), 'k');
% title('JY901 quat');
% xlabel('Time (s)');
% ylabel('quaternion');
% legend('1', '2', '3','4');
% hold off;

%% End of script