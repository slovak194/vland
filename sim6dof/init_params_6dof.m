% get_symgen_step_6dof_acado
dt = 0.01;

g = 9.80665; % Gravity

a = 0.5; % Distance from CG to bottom
b = 0.5; % Distance from CG to top

h = a+b; % Height
r = 0.025; % Radius

rho = 1000;
m = rho*h*pi*(r^2);
% m = 1;

J_e_xx = (1/12)*m*(3*r^2 + h^2);
J_e_yy = (1/12)*m*(3*r^2 + h^2);
J_e_zz = (1/2)*m*r^2;

J_e = diag([J_e_xx, J_e_yy, J_e_zz]); % Inertia tensor for a cilinder
J_e_inv = inv(J_e);

Xthrust_e = [0; 0; -a];
Xwind_e = [0; 0; 0];

p = [a; b; g; m; J_e(:); J_e_inv(:); Xthrust_e(:); Xwind_e(:)];

%% MPC params

n_X = 13;
n_u = 3;

N = 50;

lateral_thrust_lim = 0.5*m*g;
longitudal_thrust_up_lim = 2*m*g;
longitudal_thrust_down_lim = 0.5*m*g;

%% ACADO system
EXPORT = 1;
acado_root = '/home/slovak/vertical_landing/ACADOtoolkit';

codegen_folder = '/home/slovak/vertical_landing/generated';
