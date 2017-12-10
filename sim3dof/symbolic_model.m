clc
clear

problem = get_problem();

syms(problem.X) % state
syms(problem.u) % controls

syms(problem.dd) % Second derivatives
syms(problem.lX) % local state
syms(problem.p) % parameters

X = [x; y; psi; x_dot; y_dot; psi_dot];
u = [F; theta];
p = [I; a; g; m];

Fx = -m*g*sin(psi) + F*cos(theta);
Fy = -m*g*cos(psi) + F*sin(theta);
M = -F*a*sin(theta);

E_Fx = m * ax == Fx;
E_Fy = m * ay == Fy;
E_M = I * psi_ddot == M;

ax = solve(E_Fx, ax);
ay = solve(E_Fy, ay);

local_X = [ax; ay];

x_ddot = ax*cos(psi) - ay*sin(psi);
y_ddot = ax*sin(psi) + ay*cos(psi);
psi_ddot = solve(E_M, psi_ddot);

dX = [...
    x_dot; ...
    y_dot; ...
    psi_dot; ...
    x_ddot; ...
    y_ddot; ...
    psi_ddot ...
    ];

dX = simplify(dX);
pretty(dX);

%%
delete('get_symgen_*');
file_name = 'get_symgen_step';
get_symgen_dX = matlabFunction(dX, local_X, ...
    'File', file_name, ...
    'Optimize', false, ...
    'Vars', {dt, X, u, p}, ...
    'Outputs',{'dX','local_X'});

file_path = which(file_name);
file_str = fileread(file_path);

file_str = strrep(file_str, 'in2', 'X');
file_str = strrep(file_str, 'in3', 'u');
file_str = strrep(file_str, 'in4', 'p');

fid = fopen(file_path, 'w');
fprintf(fid, '%s', file_str);
fclose(fid);

rehash;
