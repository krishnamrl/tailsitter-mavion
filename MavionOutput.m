function y = MavionOutput(x,u,mavion)
%function y = MavionOutput(x,u)
% MAVION_OUTPUT
% x = [p; v; quat; omega]  
% u = [msp1; msp2; delta1; delta2] 
%   y = [p; v; quat; omega]

px = x(1);
py = x(2);
pz = x(3);
vx = x(4);
vy = x(5);
vz = x(6);
psi = x(7);
theta = x(8);
phi = x(9);
omega_x = x(10);
omega_y = x(11);
omega_z = x(12);

y = [px py pz vx vy vz psi theta phi omega_x omega_y omega_z]';

end
