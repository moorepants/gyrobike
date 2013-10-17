% This script shows how to simulate the nonlinear gyro bike model. The first
% simulation is an open loop simulation with no input torques. The second is
% an open loop simulation with an input steer torque applied given a
% function.

% Load in the constants.
constants = par_text_to_struct('gyrobike_constants_without_rider.txt');

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

% Define the right hand side to the ODEs as an anonymous function.
right_hand_side = @(time, states) gyrobike_rhs(time, states, constants);

% Intergrate the equations of motion.
[time, states] = ode45(right_hand_side, time, initial_conditions);

% Plot the results.
figure(1)

title('Simulation with no input torques.')

subplot(4, 1, 1)
plot(time, -constants.rr * states(:, 11)) % speed
ylabel('Bicycle Speed [m/s]')

subplot(4, 1, 2)
plot(time, states(:, 13) / rpm2radps)
ylabel('Flywheel Rate [rpm]')

subplot(4, 1, 3)
plot(time, rad2deg(states(:, [3, 4, 5, 7])))
ylabel('Angle [deg]')
legend('Yaw Angle', 'Roll Angle', 'Pitch Angle', 'Steer Angle')

subplot(4, 1, 4)
plot(time, rad2deg(states(:, [10, 12])))
ylabel('Angular Rate [deg/s]')
legend('Roll Rate', 'Steer Rate')

% Now simulate with an applied steering torque.

% Specify a torque function.
input_frequency = 1.0 * 2.0 * pi; % rad/s
torques.T4 = @(time, states, constants) 5.0 * sin(input_frequency * time); % Nm

% Define the right hand side to the ODEs as an anonymous function.
right_hand_side = @(time, states) gyrobike_rhs(time, states, constants, ...
                                               torques);

% Intergrate the equations of motion.
[time, states] = ode45(right_hand_side, time, initial_conditions);

figure(2)

title('Simulation with an applied steering torque.')

subplot(5, 1, 1)
plot(time, torques.T4(time, 0, 0))
ylabel('Steer Torque [Nm]')

subplot(5, 1, 2)
plot(time, -constants.rr * states(:, 11)) % speed
ylabel('Bicycle Speed [m/s]')

subplot(5, 1, 3)
plot(time, states(:, 13) / rpm2radps)
ylabel('Flywheel Rate [rpm]')

subplot(5, 1, 4)
plot(time, rad2deg(states(:, [3, 4, 5, 7])))
ylabel('Angle [deg]')
legend('Yaw Angle', 'Roll Angle', 'Pitch Angle', 'Steer Angle')

subplot(5, 1, 5)
plot(time, rad2deg(states(:, [10, 12])))
ylabel('Angular Rate [deg/s]')
legend('Roll Rate', 'Steer Rate')
