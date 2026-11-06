%% CEFIRO | Experimental 1-DOF swing-up + LQR capture (simulation only)
% This models a single roll axis with assumed pendulum inertia/damping.
% It is NOT a validated 3-DOF gimbal or the actual ESC/motor response.
% Requires Control System Toolbox for lqr().
clear; close all; clc;

J = 0.03378;         % kg m^2: provisional effective roll inertia
kg = 0.63847;       % N m / rad near upright: positive destabilizing stiffness
b = 0.05;           % N m s / rad: approximate gimbal damping
T_move = 10.0;      % s: planned inversion time
T_stop = 15.0;      % s
Ts = 0.002;         % s
Kp = 1.1;           % N m/rad: nonlinear trajectory tracking
Kd = 0.40;          % N m s/rad: nonlinear trajectory tracking
tau_swing_max = 1.1;
tau_lqr_max = 0.75;

A = [0, 1; kg / J, -b / J];
B = [0; 1 / J];
Q = diag([1 / deg2rad(5)^2, 1 / deg2rad(30)^2]);
R = 1 / (0.1 ^ 2);
K = lqr(A, B, Q, R);
fprintf('Local roll LQR gain = [%.6f, %.6f]\n', K(1), K(2));

t = 0:Ts:T_stop;
N = numel(t);
x = zeros(2,N);
x(:,1) = [pi; 0];           % Hanging equilibrium (180 deg)
tau = zeros(1,N);
reference = zeros(1,N);
phase = zeros(1,N);        % 0 = inversion, 1 = local LQR
capture_angle = deg2rad(35);
capture_rate = deg2rad(95);
in_lqr = false;

for i=1:N-1
    theta = x(1,i);
    omega = x(2,i);
    s = min(1,max(0,t(i)/T_move));
    blend = 10*s^3 - 15*s^4 + 6*s^5;
    d_blend = (30*s^2 - 60*s^3 + 30*s^4)/T_move;
    dd_blend = (60*s - 180*s^2 + 120*s^3)/T_move^2;
    theta_d = pi * (1-blend);
    omega_d = -pi * d_blend;
    alpha_d = -pi * dd_blend;
    reference(i) = theta_d;
    if ~in_lqr && abs(theta) < capture_angle && abs(omega) < capture_rate
        in_lqr = true;
    end
    if in_lqr
        torque = -K * [theta; omega];
        torque = max(-tau_lqr_max, min(tau_lqr_max, torque));
        phase(i) = 1;
    else
        torque = J*alpha_d + b*omega_d - kg*sin(theta) ...
               - Kp*(theta-theta_d) - Kd*(omega-omega_d);
        torque = max(-tau_swing_max, min(tau_swing_max, torque));
    end
    tau(i) = torque;
    % Integrate the simplified nonlinear pendulum (semi-implicit Euler).
    alpha = (kg*sin(theta) - b*omega + torque)/J;
    x(2,i+1) = omega + Ts*alpha;
    x(1,i+1) = theta + Ts*x(2,i+1);
end
reference(end) = reference(end-1);
tau(end) = tau(end-1);
phase(end) = phase(end-1);

figure('Color','w','Name','CEFIRO experimental 1DOF reorientation');
tiledlayout(3,1,'TileSpacing','compact');
nexttile; plot(t,rad2deg(x(1,:)),'LineWidth',1.5); hold on;
plot(t,rad2deg(reference),'--','LineWidth',1.2);
yline(0,':'); ylabel('Roll [deg]'); grid on; legend('Measured','Planned','Upright');
title('Simplified nonlinear inversion and capture (NOT Gazebo validation)');
nexttile; plot(t,rad2deg(x(2,:)),'LineWidth',1.5);
ylabel('Roll rate [deg/s]'); grid on;
nexttile; plot(t,tau,'LineWidth',1.5); hold on; stairs(t,phase*0.25,'--');
ylabel('Torque [N m]'); xlabel('Time [s]'); grid on;
legend('Torque command','0.25 = local LQR phase');
fprintf('Final angle %.4f deg, rate %.4f deg/s\n', ...
    rad2deg(x(1,end)),rad2deg(x(2,end)));
