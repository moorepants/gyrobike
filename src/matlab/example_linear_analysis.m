%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script shows how to work with the linear gyrobike model. It starts by
% creating a model about an equilibrium point and checking to see if it is
% stable. Secondly, the eigenvalues are computed for a range of bicycle and
% flywheel speeds. And finally the linear model is simulated to show its
% open loop response. A controller can be developed by scripting using the
% control toolbox or by adding this model to Simulink.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% RPM to rad/s conversion factor.
rpm_to_radpsec = 1.0 / 60.0 * 2.0 * pi;

% Load in the constants.
constants = par_text_to_struct('gyrobike_constants_without_rider.txt');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate a linear model. %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define the equilibrium point.
bicycle_speed = 0.5; % m/s
flywheel_rotational_speed = -1000.0 * rpm_to_radpsec; % rad/s

% Generate a linear model about the equilibrium point.
gyrobike = gyrobike_linear(bicycle_speed, flywheel_rotational_speed, ...
                           constants);

% Check if the system is stable.
eigenvalues = eig(gyrobike);

if any(real(eigenvalues) > 0.0)
    display('This system has an unstable eigenvalue!')
    display(eigenvalues)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the eigenvalues for a range of bicycle speeds. %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Specify a range of bicycle speeds in m/s and a single flywheel rate in
% rad/s.
bicycle_speeds = linspace(0.0, 10.0, 100);
flywheel_rotational_speed = -1000.0 * rpm_to_radpsec;

% Initialize the eigenvalue container.
eigenvalues = zeros(length(bicycle_speeds), size(gyrobike.A, 1));

% Compute the eigenvalues for each bicycle speed.
for i=1:length(bicycle_speeds)
    gyrobike = gyrobike_linear(bicycle_speeds(i), ...
                               flywheel_rotational_speed, constants);
    evals = eig(gyrobike);
    eigenvalues(i, :) = evals;
end

% Plot the results.
figure(1)
plot(bicycle_speeds, real(eigenvalues), 'k.')
title(sprintf('Flywheel speed = %1.0f rpm', ...
              flywheel_rotational_speed / rpm_to_radpsec))
xlabel('Speed [m/s]')
ylabel('Eigenvalue Real Part [1/s]')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the eigenvalues for a range of flywheel speeds. %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Give a range of flywheel rates in RPM.
flywheel_rates = linspace(0.0, -10000.0, 100);
bicycle_speed = 0.5; % m/s

% Initialize the eigenvalue container.
eigenvalues = zeros(length(flywheel_rates), size(gyrobike.A, 1));

% Compute the eigenvalues of the system for each flywheel rate.
for i=1:length(flywheel_rates)
    gyrobike = gyrobike_linear(bicycle_speed, ...
                               rpm_to_radpsec * flywheel_rates(i), ...
                               constants);
    evals = eig(gyrobike);
    eigenvalues(i, :) = evals;
end

% Plot the results.
figure(2)
plot(flywheel_rates, real(eigenvalues), 'k.')
title(sprintf('Bicycle speed = %1.2f m/s', bicycle_speed))
xlabel('Flywheel Rate [RPM]')
ylabel('Eigenvalue Real Part [1/s]')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulate the linear system. %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define all of the initial conditions.
initial_pitch_angle = steer_axis_tilt(constants.rf, constants.rr, ...
                                      constants.d1, constants.d3, ...
                                      constants.d2); % rad
initial_roll_rate = 0.5; % rad/s
initial_bicycle_speed = 0.5; % m/s
initial_rear_wheel_rate = -initial_bicycle_speed / constants.rr; % rad/s
rpm_to_radpsec = 1.0 / 60.0 * 2.0 * pi;
initial_flywheel_rate = -5000.0 * rpm_to_radpsec; % rad/s

initial_conditions = zeros(13, 1);
initial_conditions(5) = initial_pitch_angle;
initial_conditions(10) = initial_roll_rate;
initial_conditions(11) = initial_rear_wheel_rate;
initial_conditions(13) = initial_flywheel_rate;

% Create a time vector to integrate over.
start_time = 0.0;
time_step = 0.01;
final_time = 5.0;

time = start_time:time_step:final_time;

% Generate the linear model.
gyrobike = gyrobike_linear(initial_bicycle_speed, initial_flywheel_rate, ...
                           constants);

% Set inputs to zero.
input_torques = zeros(length(time), size(gyrobike.B, 2));

% Simulatue.
outputs = lsim(gyrobike, input_torques, time, initial_conditions);

% Plot time trajectories.
figure(3)

title('Simulation with no input torques.')

subplot(4, 1, 1)
plot(time, -constants.rr * outputs(:, 15))
ylabel('Bicycle Speed [m/s]')

subplot(4, 1, 2)
plot(time, outputs(:, 18) / rpm_to_radpsec)
ylabel('Flywheel Rate [rpm]')

subplot(4, 1, 3)
plot(time, rad2deg(outputs(:, [3, 4, 5, 7])))
ylabel('Angle [deg]')
legend('Yaw Angle', 'Roll Angle', 'Pitch Angle', 'Steer Angle')

subplot(4, 1, 4)
plot(time, rad2deg(outputs(:, [13, 16])))
ylabel('Angular Rate [deg/s]')
legend('Roll Rate', 'Steer Rate')
