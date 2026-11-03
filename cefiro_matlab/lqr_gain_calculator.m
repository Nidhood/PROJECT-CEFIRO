% Define system parameters 
Jx = 0.03378; 
Jy = 0.02444; 
Jz = 0.05613;

% System Matrix A (6x6)
A = [0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0];

% Input Matrix B (6x3)
B = [   0      0      0;
    0      0      0;
    0      0      0;
    1/Jx      0      0;
    0   1/Jy      0;
    0      0   1/Jz];

% Output Matrix C (6x6)
C = eye(6);

% State Weighting Matrix Q (6x6)
Q = eye(6);

% Control Weighting Matrix R (3x3 matching 3 inputs)
R = 0.5 * eye(3);

% Calculate LQR gain matrix K and solution S to the Algebraic Riccati Equation
K = lqr(A, B, Q, R);

% Display the resulting gain matrix K
disp('LQR Gain Matrix K:');
disp(K);