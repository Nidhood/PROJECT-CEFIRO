%% CEFIRO - Local LQR attitude stabilization about the upright equilibrium
% Requires Control System Toolbox.
% All comments are in English.
% IMPORTANT: Preliminary model, NOT validated for real propeller operation.
% The real gimbal requires a coupled multibody linearization.
clear; clc; close all;

%% 1) Nominal model parameters (SI units)
g = 9.81;
% Drone-only inertias from PROJECT-CEFIRO. These are NOT the equivalent
% inertias of the assembled moving gimbal; replace with identified values.
J_eff = diag([0.03378, 0.02444, 0.05613]); % kg*m^2 [roll pitch yaw]

% Effective destabilizing gravitational stiffness [N*m/rad].
% For a single ideal inverted pivot: k_g = m_moving*g*h_up.
% The chosen values are ILLUSTRATIVE ONLY; do not treat them as measured.
m_moving_roll  = 1.627; % kg, provisional: drone only
m_moving_pitch = 1.627; % kg, provisional: drone only
h_roll_up  = 0.040; % m, ASSUMED COM above roll pivot
h_pitch_up = 0.040; % m, ASSUMED COM above pitch pivot
Kgrav = diag([m_moving_roll*g*h_roll_up, ...
              m_moving_pitch*g*h_pitch_up, 0]); % N*m/rad
% A real multibody gravitational Hessian may have off-diagonal entries.

Dvisc = diag([0.010, 0.010, 0.015]); % N*m*s/rad, ASSUMED damping

%% 2) Linearized plant: J*qdd = Kgrav*q - D*qdot + tau
% State order: [roll pitch yaw roll_rate pitch_rate yaw_rate].
A = [zeros(3), eye(3);
     J_eff\Kgrav, -J_eff\Dvisc];
B = [zeros(3); J_eff\eye(3)];
C = eye(6);
D = zeros(6,3);
sys = ss(A,B,C,D);

%% 3) Model checks
fprintf('\n--- CEFIRO linearized model ---\n');
disp('A ='); disp(A);
disp('B ='); disp(B);
open_poles = eig(A);
disp('Open-loop poles ='); disp(open_poles);
n_ctrl = rank(ctrb(A,B));
fprintf('Controllability rank: %d / %d\n', n_ctrl, size(A,1));
assert(n_ctrl == size(A,1), 'The selected model is not fully controllable.');

%% 4) LQR tuning with physically interpretable limits
% Typical tolerances for INITIAL simulation tuning (not safety ratings).
angle_scale = deg2rad([5 5 10]); % rad [roll pitch yaw]
rate_scale  = deg2rad([30 30 45]); % rad/s
% Desired torque scale: tune to validated motor authority.
torque_scale = [0.10 0.10 0.05]; % N*m, ASSUMED; not measured

Q = diag(1./[angle_scale rate_scale].^2);
R = diag(1./torque_scale.^2);
[K,S,closed_poles] = lqr(A,B,Q,R);
Acl = A-B*K;

disp('Q ='); disp(Q);
disp('R ='); disp(R);
disp('LQR gain K (3x6) ='); disp(K);
disp('Closed-loop poles ='); disp(closed_poles);
fprintf('Largest closed-loop pole real part: %.6f\n', max(real(closed_poles)));

%% 5) Local linear simulation from a manually captured orientation
% Initial small displacement relative to upright equilibrium.
x0 = [deg2rad(4); deg2rad(-3); deg2rad(6); 0; 0; 0];
tspan = linspace(0,5,1001);
% No motor saturation in this simulation. This checks the mathematical
% linear closed loop only, NOT physical feasibility.
[t,x] = ode45(@(t,x) Acl*x,tspan,x0);
tau = -(K*x.').';

figure('Name','CEFIRO LQR: attitude','Color','w');
plot(t,rad2deg(x(:,1:3)),'LineWidth',1.5);
grid on; xlabel('Time [s]'); ylabel('Angle error [deg]');
legend('Roll','Pitch','Yaw','Location','best');
title('Linear local stabilization about the desired upright equilibrium');

figure('Name','CEFIRO LQR: control effort','Color','w');
plot(t,tau,'LineWidth',1.5);
grid on; xlabel('Time [s]'); ylabel('Requested generalized torque [N*m]');
legend('Roll torque','Pitch torque','Yaw torque','Location','best');
title('Ideal LQR torque demands (without saturation)');

max_torque_requested = max(abs(tau),[],1);
fprintf('\nPeak requested torque [roll pitch yaw], N*m:\n');
disp(max_torque_requested);
fprintf('Check these against measured motor/mixer limits before hardware use.\n');

%% 6) Export K to the current ros2_control row-major YAML layout
fprintf('\n--- Paste into gains.control_gains.k_matrix (row-major) ---\n');
fprintf('k_matrix: [\n');
for i=1:3
    fprintf('  ');
    fprintf('%.9g',K(i,1));
    for j=2:6
        fprintf(', %.9g',K(i,j));
    end
    if i<3, fprintf(',\n'); else, fprintf('\n'); end
end
fprintf(']\nk_rows: 3\nk_cols: 6\n');

%% 7) Warning: consistency between the controller and the real plant
% Using these gains in ROS 2 requires the SAME state and torque units,
% gravitational model, motor allocation convention and equilibrium.
% The existing C++ observer uses a gravity-free A model: update its
% dynamics too, rather than copying K alone.
% Verify sensor frames, rate estimation, motor signs and thrust units.
% A state-machine interlock is needed for the manual capture procedure.
