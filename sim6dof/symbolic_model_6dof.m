clc
clear

syms m g a b dt real

J_e = sym('J_e', [3, 3], 'real');
J_e_inv = sym('J_e_inv', [3, 3], 'real');

R_I = sym('R_I_', [3, 1], 'real');
V_I = sym('V_I_', [3, 1], 'real');
w_e = sym('w_e_', [3, 1], 'real');
L = sym('L_', [4, 1], 'real');

X = [...
    R_I; ...
    V_I; ...
    w_e; ...
    L;...
    ];

Fthrust_e = sym('Fthrust_e', [3, 1], 'real');
Xthrust_e = sym('Xthrust_e', [3, 1], 'real');
u = [Fthrust_e];

Fmg_I = [0; 0; -m*g];
Fthrust_I = quatrotate_sym(quatconj_sym(L), Fthrust_e);
Mthrust_e = cross(Xthrust_e, Fthrust_e);

E_Fi_I = Fthrust_I + Fmg_I;
E_Mi_e = Mthrust_e;

dR_I = V_I;
dV_I = E_Fi_I/m;

dw_e = J_e_inv*(E_Mi_e - cross(w_e, (J_e*w_e)));
dL = (1/2)*quatmultiply_sym(L, [0; w_e]);

p = [a; b; g; m; J_e(:); J_e_inv(:); Xthrust_e(:)];

dX = [...
    dR_I; ...
    dV_I; ...
    dw_e; ...
    dL; ...
    ];

Debug = [...
    E_Fi_I; ...
    E_Mi_e; ...
    dw_e; ...
    dL; ...
    ];

dX = simplify(dX);
Debug = simplify(Debug);

%%
file_name = 'get_symgen_step_6dof';
get_symgen_dX = matlabFunction(dX, Debug, ...
    'File', file_name, ...
    'Optimize', true, ...
    'Vars', {dt, X, u, p}, ...
    'Outputs',{'dX', 'Debug'});

file_path = which(file_name);
file_str = fileread(file_path);

file_str = strrep(file_str, 'in2', 'X');
file_str = strrep(file_str, 'in3', 'u');
file_str = strrep(file_str, 'in4', 'p');
file_str = strrep(file_str, [';', newline], ';');
file_str = strrep(file_str, ';', [';', newline]);

fid = fopen(file_path, 'w');
fprintf(fid, '%s', file_str);
fclose(fid);

rehash;

disp('Done!')
% edit(file_name)

%%

% x = X;
% syms t w xa
% 
% file_name = 'get_symgen_step_6dof_acado';
% get_symgen_dX = matlabFunction(dX, ...
%     'File', file_name, ...
%     'Optimize', false, ...
%     'Vars', {t, x, xa, u, p, w}, ...
%     'Outputs',{'f'});
% 
% file_path = which(file_name);
% file_str = fileread(file_path);
% 
% file_str = strrep(file_str, 'in2', 'x');
% file_str = strrep(file_str, 'in4', 'u');
% file_str = strrep(file_str, 'in5', 'p');
% file_str = strrep(file_str, [';', newline], ';');
% file_str = strrep(file_str, ';', [';', newline]);
% 
% fid = fopen(file_path, 'w');
% fprintf(fid, '%s', file_str);
% fclose(fid);
% 
% rehash;
% edit(file_name)