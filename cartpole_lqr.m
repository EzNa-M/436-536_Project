% State order used in this file:
% z = [x; xdot; theta; thetadot]
% Input: motor voltage v

sys = cartpole();

% Motor and cart parameters from UnderGradProject_MME436
Ra    = 2.6;          % armature resistance [ohm]
eta_g = 0.69;         % gearbox efficiency
eta_m = 0.69;         % motor efficiency
kg    = 3.71;         % gearbox ratio
kf    = 7.68e-3;      % torque constant [N*m/A], called kt in document
kb    = 7.68e-3;      % back-emf constant [V*s/rad]
rmp   = 6.35e-3;      % motor pinion radius [m]
Jm    = 4.0e-6;       % motor rotor inertia [kg*m^2]
mc    = 0.057;        % cart mass [kg], as listed in document
g     = 9.81;         % gravitational acceleration [m/s^2]

% Pendulum parameters: supply your measured/model values
mp = 0.037;            % pendulum mass [kg]
L  = 0.6;            % full pendulum length [m]
bp = 0;            % pivot viscous damping [N*m*s/rad]

assert(all(isfinite([mp, L, bp])), ...
    'Set mp, L, and bp before running the linearization.');

% Motor-integrated coefficients
phi = eta_g*eta_m*kg*kf/(Ra*rmp);

% M follows the acceleration coefficient in your supplied equation
M = (mc + mp)*(phi*Ra*Jm*kg/(rmp*eta_m*kf));

Kem = kb*kg*phi/rmp;  % avoid naming this K: K is used for LQR gain
H   = mp*L/2;
Ip  = mp*L^2/3;
G   = mp*g*L/2;

Delta = M*Ip - H^2;

% Track and voltage limits
xmin = 0;
xmax = 1;             % idealized allowable cart-position range [m]
Vmax = 6;             % voltage limits: -6 V to +6 V

% Equilibrium: cart centered, pendulum upright
xtilde = [0.5; 0; pi/2; 0];
utilde = 0;

t = 0:0.01:20;

% Initial condition: centered cart, small angular disturbance
x0 = [0.5; 0; pi/2 + 0.1; 0];

assert(x0(1) > xmin && x0(1) < xmax, ...
    'Initial cart position must be inside the track limits.');

% Linearized model in deviation coordinates:
% delta_z_dot = A*delta_z + B*delta_v
%
% State order: [x; xdot; theta; thetadot]
A = [0,  1,             0,             0;
     0, -Ip*Kem/Delta, -H*G/Delta,     H*bp/Delta;
     0,  0,             0,             1;
     0,  H*Kem/Delta,   M*G/Delta,    -M*bp/Delta];

B = [0;
     Ip*phi/Delta;
     0;
    -H*phi/Delta];

% Outputs: cart position and pendulum angle
C = [1, 0, 0, 0;
     0, 0, 1, 0];

D = zeros(2,1);

% Voltage-input linear state-space model
sys_lin = ss(A, B, C, D);




% cost matrices
Q = [1 0 0 0;   % position of the cart
    0 10 0 0;    % velocity of the cart
    0 0 10 0;   % angle of the pendulum
    0 0 0 100]; % angular velocity of the pendulum
R = 0.0001;     % force on the cart

% static state feedback using LQR
K = lqr(A,B,Q,R);

% simulate the system
x = sys.static_state_feedback(t,x0,K,xtilde,utilde);

% visualize the dynamics
sys.draw(x);