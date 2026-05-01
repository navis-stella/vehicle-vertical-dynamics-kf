function [x, y_err] = kalman_filter(u, y, kFv, kR, kFh, dR, dFv, dFh, ...
                       m1, m2, m3, m4, m5, Jphi, Jtheta, ...
                       b, lv, lh, R, Q, P_0, x_0, T_abt)
% KALMAN_FILTER  Discrete-time Kalman Filter for 7-DOF vehicle vertical dynamics.
%
% Implements the predict-update cycle of a linear discrete Kalman Filter.
% Designed as a MATLAB Function block for Simulink — uses persistent variables
% to maintain state across time steps.
%
% INPUTS:
%   u      - (8×1) Input vector [u1;u2;u3;u4;du1;du2;du3;du4] — road excitation
%   y      - (7×1) Measurement vector [z1;z2;z3;z4;z5;Phi;Theta]
%   kFv, kR, kFh, dR, dFv, dFh    - Spring/damper parameters [N/m, Ns/m]
%   m1..m5, Jphi, Jtheta           - Mass and inertia parameters [kg, kgm^2]
%   b, lv, lh                      - Vehicle geometry [m]
%   Q      - (14×14) Process noise covariance matrix
%   R      - (7×7)  Measurement noise covariance matrix
%   P_0    - (14×14) Initial state error covariance
%   x_0    - (14×1) Initial state estimate
%   T_abt  - Sampling time [s]
%
% OUTPUTS:
%   x      - (14×1) Corrected state estimate at current time step
%   y_err  - (7×1)  Innovation residual (y - C*x_predicted) — for diagnostics
%
% STATE VECTOR (14 states):
%   x = [z1, z2, z3, z4, z5, Phi, Theta, dz1, dz2, dz3, dz4, dz5, dPhi, dTheta]'
%
% DISCRETIZATION:
%   Forward Euler: Ad = I + Ts*Ac,  Bd = Ts*Bc
%   Valid for small sampling times (Ts = 0.01 s used here).
%
% NOTE:
%   The persistent variables x_est and P_est are reset automatically when
%   the function is first called (e.g. at the start of each simulation run).

persistent x_est P_est K
if isempty(x_est)
   x_est = x_0;
   P_est = P_0;
end

%% Discrete Kalman filter time update equations

x = x_est;

% Modellgleichungen in Matrizenform  
A_c = [0                0               0                0               0                         0                        0                              1                0               0                0               0                         0                        0;
       0                0               0                0               0                         0                        0                              0                1               0                0               0                         0                        0;
       0                0               0                0               0                         0                        0                              0                0               1                0               0                         0                        0;
       0                0               0                0               0                         0                        0                              0                0               0                1               0                         0                        0;
       0                0               0                0               0                         0                        0                              0                0               0                0               1                         0                        0;  
       0                0               0                0               0                         0                        0                              0                0               0                0               0                         1                        0;
       0                0               0                0               0                         0                        0                              0                0               0                0               0                         0                        1;
       -(kR+kFv)/m1     0               0                0               kFv/m1                    -b*kFv/(2*m1)            lv*kFv/m1                      -(dR+dFv)/m1     0               0                0               dFv/m1                    -b*dFv/(2*m1)            lv*dFv/m1;
       0                -(kR+kFv)/m2    0                0               kFv/m2                    b*kFv/(2*m2)             lv*kFv/m2                      0                -(dR+dFv)/m2    0                0               dFv/m2                    b*dFv/(2*m2)             lv*dFv/m2;
       0                0               -(kR+kFh)/m3     0               kFh/m3                    -b*kFh/(2*m3)            -lh*kFh/m3                     0                0               -(dR+dFh)/m3     0               dFh/m3                    -b*dFh/(2*m3)            -lh*dFh/m3;
       0                0               0                -(kR+kFh)/m4    kFh/m4                    b*kFh/(2*m4)             -lh*kFh/m4                     0                0               0                -(dR+dFh)/m4    dFh/m4                    b*dFh/(2*m4)             -lh*dFh/m4;
       kFv/m5           kFv/m5          kFh/m5           kFh/m5          -2*(kFv+kFh)/m5           0                        2*(kFh*lh-kFv*lv)/m5           dFv/m5           dFv/m5          dFh/m5           dFh/m5          -2*(dFv+dFh)/m5           0                        2*(dFh*lh-dFv*lv)/m5;
       -b*kFv/(2*Jphi)  b*kFv/(2*Jphi)  -b*kFh/(2*Jphi)  b*kFh/(2*Jphi)  0                         -b^2*(kFh+kFv)/(2*Jphi)  0                              -b*dFv/(2*Jphi)  b*dFv/(2*Jphi)  -b*dFh/(2*Jphi)  b*dFh/(2*Jphi)  0                         -b^2*(dFh+dFv)/(2*Jphi)  0;
       lv*kFv/Jtheta    lv*kFv/Jtheta   -lh*kFh/Jtheta   -lh*kFh/Jtheta  2*(kFh*lh-kFv*lv)/Jtheta  0                        -2*(lv^2*kFv+lh^2*kFh)/Jtheta  lv*dFv/Jtheta    lv*dFv/Jtheta   -lh*dFh/Jtheta   -lh*dFh/Jtheta  2*(kFh*lh-kFv*lv)/Jtheta  0                       -2*(lv^2*dFv+lh^2*dFh)/Jtheta]; 

B_temp = zeros(7,8);
B_c = [B_temp;
       kR/m1  0      0      0      dR/m1  0      0      0;
       0      kR/m2  0      0      0      dR/m2  0      0;
       0      0      kR/m3  0      0      0      dR/m3  0;
       0      0      0      kR/m4  0      0      0      dR/m4;
       0      0      0      0      0      0      0      0;
       0      0      0      0      0      0      0      0;
       0      0      0      0      0      0      0      0]; 
  
% Ad   = diag([ones(1,14)]) + T_abt * A_c;      % Umrechnung in zeitdiskrete Form
Ad = expm(A_c * T_abt);
Bd   = T_abt * B_c;                              % Umrechnung in zeitdiskrete Form


x_ = Ad * x + Bd * u;

C = [1  0  0  0  0  0  0  0  0  0  0  0  0  0;
     0  1  0  0  0  0  0  0  0  0  0  0  0  0;
     0  0  1  0  0  0  0  0  0  0  0  0  0  0;
     0  0  0  1  0  0  0  0  0  0  0  0  0  0;
     0  0  0  0  1  0  0  0  0  0  0  0  0  0;
     0  0  0  0  0  1  0  0  0  0  0  0  0  0;
     0  0  0  0  0  0  1  0  0  0  0  0  0  0];
 

%% Discrete Kalman filter measurement update equations
P_  = Ad * P_est * Ad' + Q;
K   = P_ * C'/( C * P_ * C' + R);
x   = x_ +  K * ( y - C * x_);
% P_est  = P_ - K *( C * P_ * C' + R)* K';
P_est = (eye(14) - K*C) * P_ * (eye(14) - K*C)' + K*R*K'; % Joseph stabilized form
x_est  = x;
y_err = y - C * x_;
