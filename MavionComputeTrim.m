%% Struct parameter setup
mavion.gravity = 9.81;
mavion.CdT = 0.01; % loss of prop efficiency due to wing in propwash
mavion.ClT = 0.5; % propwash-induced lift coefficient
mavion.CldeltaT = 0.5; % flap lift due to propwash-induced airspeed
mavion.CldeltaV = 0.5; % flap lift due to airspeed along 0-lift line
mavion.ClV = 0.5; % wing lift coefficient
mavion.CdV = 0.01; % wing drag coefficient
mavion.lTy = 0.105; % abs distance along b_y between vehicle cg and each motor
mavion.cmuT = 1; % pitch moment coefficient due to thrust
mavion.ldeltay = 0.123; % abs distance along b_y between vehicle cg and each flap centre
mavion.ldeltax = 0.0525;  % distance from b_y to ac of both flaps
mavion.cmu = 0.85; % propeller torque efficiency ; 
mavion.mass = 0.420;
mavion.alpha0 = 0; % angle b/w alpha_x and b_x
mavion.alphaT = 0; % angle b/w thrust dir and b_x
mavion.cT = 0.1207; % thrust coefficient
mavion.J = 1; % moment of inertia tensor
mavion.dia = 0.178; %m 7; % inch
mavion.pitch = 0.127; %m 5; %inch

%% Trim computation
% Desired trim state
% Hover
% x_trim = zeros(12,1);
% x_trim(8) = pi/2;

% Constant velocity foward flight
x_trim = [13.32 0 -1 6.66 0 0 0 40*pi/180 0 0 0 0]';

% Define the system dynamics function
f = @(u) MavionDynamics(x_trim, u, mavion);

% Give the solver an initial guess for input
u0 = [-700; 700; 40*pi/180; 40*pi/180];

% Run the solver
trim_inputs = fsolve(f, u0);

% Display the results
disp('Trimmed Input Values:');
disp(trim_inputs);