%% 
clear

IMU = imuSensor('accel-gyro-mag','SampleRate',100,'MagneticField',[16 0 27.712])

m_local=[16 0 27.712];
IMU.Accelerometer = accelparams( ...
    'MeasurementRange',117.6, ...            % m/s^2
    'Resolution',0.001795, ...               % m/s^2 / LSB
    'TemperatureScaleFactor',0.002, ...      % % / degree C 
    'TemperatureBias',0.00196, ...           % m/s^2 / degree C
    'NoiseDensity',0.003,...                 %  ‘Î…˘√‹∂»£®rad/s)/°ÃHz
     "AxesMisalignment",0.6,...              % %£®÷·∆´≤Ó£©    
     'RandomWalk',0.0005);                   % ∏ﬂÀπ∞◊‘Î…˘£®rad/s)*°ÃHz 

IMU.Magnetometer = magparams( ...
    'MeasurementRange',1200, ...             % uT
    'Resolution',0.3, ...                    % uT / LSB
    'TemperatureScaleFactor',0.1, ...        % % / degree C     
    'TemperatureBias',[0.1 0.1 0.1], ...     % uT / degree C
    'NoiseDensity',[0.6 0.6 0.9]/sqrt(100)); % uT / °ÃHz
IMU.Gyroscope =gyroparams( ...
    'MeasurementRange',8.7266, ...           % rad/s  ≤‚¡ø∑∂Œß
    'Resolution',0.00013323, ...             % (rad/s)/LSB £®“ª∏ˆ∂¡»°÷µ¥˙±Ì∂‡…Ÿrad/s£©
    'AxesMisalignment',2, ...                % %£®÷·∆´≤Ó£©
    'NoiseDensity',8.7266e-05, ...           %  ‘Î…˘√‹∂»£®rad/s)/°ÃHz
    'TemperatureBias',0.34907, ...           %£®rad/s)/°Ê
    'TemperatureScaleFactor',0.02, ...       % %/°Ê
    'AccelerationBias',0.00017809,...        % (rad/s)/(m/s^2)
    'RandomWalk',0.0005);                    % ∏ﬂÀπ∞◊‘Î…˘£®rad/s)*°ÃHz 
IMU.Magnetometer
IMU.Gyroscope
IMU.Accelerometer

%%
fs =100;
firstLoopNumSamples = fs*2;
secondLoopNumSamples = fs*2;
thirdLoopNumSampales=fs*2;
fourthLoopNumSampales=fs*3;
fifthLoopNumSampales=fs*3;
sixthLoopNumSampales=fs*8;

totalNumSamples =140*fs;

traj = kinematicTrajectory('SampleRate',fs);

accBody = zeros(totalNumSamples,3);
angVelBody = zeros(totalNumSamples,3);

angVelBody(1:end,1)=4;
angVelBody(1:end,2)=0;
angVelBody(1:end,3)=0;











[~,orientationNED,~,accNED,angVelNED] = traj(accBody,angVelBody);

[accelReadings,gyroReadings,magReadings] = IMU(accNED,angVelNED,orientationNED);

%% prepare sensor data
sampleFrequency = fs;
samplePeriod = 1/sampleFrequency;
time=(1:totalNumSamples)';time=time*samplePeriod;
Gyroscope=gyroReadings;
Accelerometer=accelReadings/9.8; % m/s^2->g
Magnetometer=magReadings/100; % uT->G

%% plot sensor data
figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (rad/s)');
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
initBeta=0.4;Beta=0.4;
initTime=5*sampleFrequency;

AHRS = MadgwickAHRS('SamplePeriod', samplePeriod, 'Beta', initBeta);
%AHRS = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 0.5);

q = zeros(length(time), 4);

for t = 1:initTime
    AHRS.Update(Gyroscope(t,:), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
    q(t, :) = AHRS.Quaternion;
end
AHRS.Beta=Beta;
for t = initTime+1:length(time)
    AHRS.Update(Gyroscope(t,:), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
    q(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ?90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

q_filter=quaternion(q);

euler_angle=eulerd(q_filter,'ZYX',"point");

% euler = quatern2euler(quaternConj(q)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure('Name', 'Euler Angles');
hold on;
plot(time, euler_angle(:,1), 'r');
plot(time, euler_angle(:,2), 'g');
plot(time, (euler_angle(:,3)), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

%% plot IMUsensorData angles as reference

eu_de=eulerd(orientationNED,'ZYX',"point");
figure('Name', 'Reference Angles');
hold on;
plot(time, eu_de(:,1), 'r');
plot(time, eu_de(:,2), 'g');
plot(time, (eu_de(:,3)), 'b');
title('Reference angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

%% 
figure('Name', 'Angles');
e1=euler_angle(:,1);
e2=eu_de(:,1);

h1=euler_angle(:,2);
h2=eu_de(:,2);

o1=euler_angle(:,3);
o2=eu_de(:,3);

e=e1-e2;
hold on

subplot(2,2,1)
plot(time,180 + JYcontinuous(e))
title('angle error')

subplot(2,2,2)
plot(time,e1,'--',time,e2,':')
title('æ≤÷π◊¥Ã¨œ¬Z÷·¡„∆Ø≤‚ ‘')
xlabel('Time (s)')
ylabel('Rotation (degrees)')
legend('‘§≤‚÷µ',' µº ÷µ')



subplot(2,2,3)
plot(time,h1,'--',time,h2,':')
title('angle2')

subplot(2,2,4)
plot(time,o1,'--',time,o2,':')
title('angle3')

hold off