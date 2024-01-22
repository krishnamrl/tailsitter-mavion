clc
clear 
close all
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

%% X dot computation
% its possible that the reference output and the constraints conflict with
% each other, and that's why theta and px are messed up and it is moving
% backwards

load('tiltbody_transition_alt.mat')
x_dot_arr = zeros(12,101);
x_dot_arr = zeros(12,101);
for i=1:101
    x = [px(i) py(i) pz(i) vx(i) vy(i) vz(i) psi(i) theta(i) phi(i) omega_x(i) omega_y(i) omega_z(i)]';
    u = [msp1(i) msp2(i) delta1(i) delta2(i)]';
    % Define the system dynamics function
    x_dot = MavionDynamics(x, u, mavion);
    x_dot_arr(:,i) = x_dot; 
end

%%
plot(t,vx,LineWidth=1)
hold on 
plot(t,x_dot_arr(4,:),LineWidth=1)
grid on
xlabel("t (s)")
legend("v_x (m/s)","a_x (m/s^2)")
hold off

figure
plot(t,theta*180/pi,LineWidth=1)
xlabel("t (s)")
ylabel("\theta (deg)")
grid on

figure
plot(t,msp1,LineWidth=1)
hold on
plot(t,msp2,LineWidth=1)
grid on
xlabel("t (s)")
legend("\omega_1 (rad/s)","\omega_2 (rad/s)")
hold off