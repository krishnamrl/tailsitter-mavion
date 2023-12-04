clc
clear
%% MAVION struct parameters definition

mavion.gravity = 9.81;
mavion.CdT = 0.01; % loss of prop efficiency due to wing in propwash
mavion.ClT = 0.5; % propwash-induced lift coefficient
mavion.CldeltaT = 0.5; % flap lift due to propwash-induced airspeed
mavion.CldeltaV = 0.5; % flap lift due to airspeed along 0-lift line
mavion.ClV = 0.5; % wing lift coefficient
mavion.CdV = 0.01; % wing drag coefficient
mavion.lTy = 0.105; % abs distance along b_y between vehicle cg and each motor
mavion.cmuT = 0; % pitch moment coefficient due to thrust
mavion.ldeltay = 0.123; % abs distance along b_y between vehicle cg and each flap centre
mavion.ldeltax = 0.0525;  % distance from b_y to ac of both flaps
mavion.cmu = 0.85; % propeller torque efficiency ; 
mavion.mass = 0.420;
mavion.alpha0 = 0; % angle b/w alpha_x and b_x
mavion.alphaT = 0; % angle b/w thrust dir and b_x
mavion.cT = 0.0950; % thrust coefficient
mavion.J = 1; % moment of inertia tensor
mavion.dia = 0.178; %m 7; % inch
mavion.pitch = 0.127; %m 5; %inch
mavion.density = 1.225; % kg/m^3

%% MPC handler config
% MPC handler init
nx = 12; % dimension of state vector
ny = 12; % dimension of output vector
nu = 4; % dimension of input vector
nlobj = nlmpc(nx,ny,nu); % create nonlinear MPC solver handle

% MPC basic control parameters
nlobj.Ts = 0.02; % sampling time equal to usual servo frequency
nlobj.PredictionHorizon = 100; % how many steps to look ahead (2 sec)
nlobj.ControlHorizon = 100; % only first 20 actions can be variable

% this is where we link the MPC handler to our dynamics model and call the custom cost function
nlobj.Optimization.ReplaceStandardCost = true;
nlobj.Model.StateFcn = "MavionDynamics";
nlobj.Optimization.CustomCostFcn = "AltCostFcn";
nlobj.Model.OutputFcn = "MavionOutput";
nlobj.Model.IsContinuousTime = true;

% setting the number of additional parameters of solver.
% we only use the MAVION structure as additional argument!
nlobj.Model.NumberOfParameters = 1;

% setting optimization solver options here!
nlobj.Optimization.SolverOptions.Algorithm = 'active-set';
nlobj.Optimization.SolverOptions.Display = 'iter';
nlobj.Optimization.UseSuboptimalSolution = true;
nlobj.Optimization.SolverOptions.MaxIterations = 8;

% this set the equality constraints of the MPC problem:
% in our case, we want all the state vector elements to be 0 except for the 
% vertical position, which should be positive; and pitch angle theta which
% should be pi/2 rad - we don't care about x and y position.
%nlobj.Optimization.CustomEqConFcn = @EqConFcn;
nlobj.Optimization.CustomEqConFcn = @(X,U,data,params) [X(end,4:7)'; X(end,8)-(pi/2); X(end,9:end)'];

% this set the inequality constraints of the MPC problem:
% in our case, we want MAVION to stay above ground all times during
% the entire flight phase.
%nlobj.Optimization.CustomIneqConFcn = @IneqConFcn;
nlobj.Optimization.CustomIneqConFcn = @(X,U,e,data,params) X(1:end-9,3);


% cost function options! I didn't play with it much, since I was happy
% having any trajectory that worked. in practice, you should tune these
% values to obtain the best and safest trajectory!
nlobj.Weights.OutputVariables = [10 0 10 10 0 0 0 10 0 0 0 0];
nlobj.Weights.ManipulatedVariables = [1 1 1 1];
nlobj.Weights.ManipulatedVariablesRate = zeros(1,4);

% Input Constraints!
nlobj.ManipulatedVariables(1).Min = -1686;
nlobj.ManipulatedVariables(2).Min = 0.1;
nlobj.ManipulatedVariables(3).Min = -30*pi/180;
nlobj.ManipulatedVariables(4).Min = -30*pi/180;
nlobj.ManipulatedVariables(1).Max = -0.1;
nlobj.ManipulatedVariables(2).Max = 1686;
nlobj.ManipulatedVariables(3).Max = 30*pi/180;
nlobj.ManipulatedVariables(4).Max = 30*pi/180;
nlobj.ManipulatedVariables(3).RateMin = -60*pi/180; % considering max speed 0.2 (sec/60deg)
nlobj.ManipulatedVariables(4).RateMax = 60*pi/180;

% initial conditions for the simulation
x0 = zeros(nx,1);
x0(3) = -1; % assume we are 1 meter above ground
x0(8) = pi/2;
u0 = [-843.1393; 843.1393; 0; 0];

% this function runs some sanity checks for us
validateFcns(nlobj,x0,u0,[],{mavion});

% in the following, we set up an initial guess for this problem since it is
% a difficult problem to converge to a solution.
nloptions = nlmpcmoveopt;
nloptions.Parameters = {mavion};

% solve the problem!!!
[~,~,info] = nlmpcmove(nlobj,x0,u0,[],[],nloptions);


%% extract the solution data

t = info.Topt';
px = info.Xopt(:,1)';
py = info.Xopt(:,2)';
pz = info.Xopt(:,3)';
vx = info.Xopt(:,4)';
vy = info.Xopt(:,5)';
vz = info.Xopt(:,6)';
psi = info.Xopt(:,7)';
theta = info.Xopt(:,8)';
phi = info.Xopt(:,9)';
omega_x = info.Xopt(:,10)';
omega_y = info.Xopt(:,11)';
omega_z = info.Xopt(:,12)';

msp1 = info.MVopt(:,1);
msp2 = info.MVopt(:,2);
delta1 = info.MVopt(:,3);
delta2 = info.MVopt(:,4);