function x_dot = MavionDynamics(x,u,mavion)
%function x_dot = MavionDynamics(x,u)

% MAVION_DYNAMICS
% x = [p; v; quat; omega];  
% u = [msp1; msp2; delta1; delta2]; 

g = mavion.gravity;
CdT = mavion.CdT; 
ClT = mavion.ClT; 
CldeltaT = mavion.CldeltaT; 
CldeltaV = mavion.CldeltaV; 
ClV = mavion.ClV;
CdV = mavion.CdV; 
lTy = mavion.lTy;
cmuT = mavion.cmuT; 
ldeltay = mavion.ldeltay; 
ldeltax =mavion.ldeltax;
cmu = mavion.cmu; 
mass = mavion.mass;
alpha0 = mavion.alpha0; 
alphaT = mavion.alphaT; 
cT = mavion.cT; 
J = mavion.J; 
dia = mavion.dia;
pitch = mavion.pitch;
rho = mavion.density;

% g = 9.81;
% CdT = 0.01; % loss of prop efficiency due to wing in propwash
% ClT = 0.5; % propwash-induced lift coefficient
% CldeltaT = 0.5; % flap lift due to propwash-induced airspeed
% CldeltaV = 0.5; % flap lift due to airspeed along 0-lift line
% ClV = 0.5; % wing lift coefficient
% CdV = 0.01; % wing drag coefficient
% lTy = 0.105; % abs distance along b_y between vehicle cg and each motor
% cmuT = 0; % pitch moment coefficient due to thrust
% ldeltay = 0.123; % abs distance along b_y between vehicle cg and each flap centre
% ldeltax = 0.0525;  % distance from b_y to ac of both flaps
% cmu = 0.85; % propeller torque efficiency 
% cT = 0.0950; % thrust coefficient
% mass = 0.420;
% alpha0 = 0; % angle b/w alpha_x and b_x
% alphaT = 0; % angle b/w thrust dir and b_x
% J = 1; % moment of inertia tensor
% dia = 0.1778; % m; 7 inch
% rho = 1.225;


msp1 = u(1); % input 1; motor 1 angular velocity
msp2 = u(2); % input 2; motor 2 angular velocity
delta1 = u(3); % input 3; port(l) side flap
delta2 = u(4); % input 4; starboard(r) side flap

p = x(1:3); % inertial frame
px = p(1); 
py = p(2);
pz = p(3); 

v = x(4:6); % inertial frame
vx = v(1);
vy = v(2);
vz = v(3);

% psi is defined as the angle between (inertial) iy and projection of by onto the horizontal plane
euler = x(7:9); % inertial frame
psi = euler(1);
theta = euler(2);
phi = euler(3);
quat_xi = Euler3212EP([psi theta phi]'); % body frame quaternion

omega = x(10:12); % body frame
eul_dot = BmatEuler321(omega)*omega;
psi_dot = eul_dot(1); 
theta_dot = eul_dot(2);
phi_dot = eul_dot(3);

Rzl_body = Euler3212C([0, -alpha0, 0]); % rot mat from body frame to zero-lift frame
Rbody_i = Euler3212C([psi, theta,  phi]); % rot mat from inertial frame to body frame 

ix = [1;0;0];
iy = [0;1;0];
iz = [0;0;1];

%% Forces 

% Thrust
v_zl = Rzl_body*Rbody_i*v; % rot mat are multiplied in the order opposite to the actual rotations (here v: inertial -> body -> zl)
alpha = alpha0 + alphaT; 

T1 = cT*(rho*((msp1*0.159)^2)*(dia^4)); % 0.159 rps = 1 rad/s
T2 = cT*(rho*((msp2*0.159)^2)*(dia^4));

falpha_T1 = [cos(alpha)*(1 - CdT); 0; sin(alpha)*(ClT - 1)]*T1; % look into the wind axes issue in transition flight
falpha_T2 = [cos(alpha)*(1 - CdT); 0; sin(alpha)*(ClT - 1)]*T2; 

% Flaps
falpha_delta1 = - [0; 0; CldeltaT*cos(alpha)*T1 + CldeltaV*norm(v)*ix'*v_zl].*delta1 ;  
falpha_delta2 = - [0; 0; CldeltaT*cos(alpha)*T2 + CldeltaV*norm(v)*ix'*v_zl].*delta2 ; 

% Wing
falpha_wing = - [CdV*ix'*v_zl; 0 ; ClV*iz'*v_zl] * norm(v);

falpha = falpha_T1 + falpha_T2 + falpha_delta1 + falpha_delta2 + falpha_wing; % zero-lift frame
%% Moments

% Moment due to motor thrust
m_T = [lTy* iz' * (Rzl_body)' * (falpha_T2 - falpha_T1) ; cmuT*(T1+T2) ; lTy* ix' * (Rzl_body)' * (falpha_T1 - falpha_T2)]; 

% Moment due to motor torque
% mu1 = -(-1)^(1)*cmu*msp1^2 ;                            
% mu2 =  -(-1)^(2)*cmu*msp2^2 ; 

% specific to thrust at that condition!
torq = 0.037; % datasheet, changes for each thrust
%torq = 0.094; % forward flight
cq = torq/(rho*((msp1*0.159)^2)*(dia^5));
arm = 0.1225;
mu1 = (-1)^(1)*cq*rho*((msp1*0.159)^2)*(dia^5)*arm;
mu2 = (-1)^(2)*cq*rho*((msp2*0.159)^2)*(dia^5)*arm;

m_mu = [cos(alphaT); 0; -sin(alphaT)] * (mu1+mu2); 

% Flap contribution 
m_delta = [ldeltay*cos(alpha0)*iz'*(falpha_delta2-falpha_delta1); ldeltax*iz'*(falpha_delta1+falpha_delta2) ; ldeltay*sin(alpha0)*iz'*(falpha_delta2-falpha_delta1)]; 

m = m_T+m_mu+ m_delta; % zero-lift frame

Rzl_i = Rzl_body*Rbody_i; % rot mat from zero-lift frame to inertial frame (Rzl_body*Rbody_i gives inertial -> zl, transpose it)

%% Dynamics 
x_dot = sym(zeros(12,1));
% xdot = [p_dot; v_dot; omega; omega_dot]

x_dot(1:3) = [vx; vy; vz];

v_dot = g*iz + mass^(-1)*Rzl_i'*falpha;
x_dot(4:6) = [v_dot(1); v_dot(2); v_dot(3)];

quat_xi_dot = 0.5*sym_quatmultiply(quat_xi',[0;omega]');  % body frame quaternion 
omega_dyn = quatdot2omega(quat_xi,quat_xi_dot);
x_dot(7:9) = [omega_dyn(1); omega_dyn(2); omega_dyn(3)];

omega_dot = J\(m - cross(omega_dyn,J*omega_dyn)); % equivalent to  inv(J)*(m - cross(omega_df,J*omega_df))
x_dot(10:12) = [omega_dot(1); omega_dot(2); omega_dot(3)]; % angular acceleration
x_dot = double(x_dot); % comment out to run linearize script
end 