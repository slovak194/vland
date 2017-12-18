clc
clear

syms m g a b dt Fx real

% J_e = sym('J_e', [3, 3], 'real');
% J_e_inv = sym('J_e_inv', [3, 3], 'real');
% 
% R_I = sym('R_I_', [3, 1], 'real');
% V_I = sym('V_I_', [3, 1], 'real');
% w_e = sym('w_e_', [3, 1], 'real');
L = sym('L_', [4, 1], 'real');

Mthrust_e = sym('Mthrust_e', [3, 1], 'real');

% 
% X = [...
%     R_I; ...
%     V_I; ...
%     w_e; ...
%     L;...
%     ];

Fthrust_e = sym('Fthrust_e', [3, 1], 'real');
Xthrust_e = sym('Xthrust_e', [3, 1], 'real');

% 
% Fwind_I = sym('Fwind_I', [3, 1], 'real');
% Fwind_e = quatrotate_sym(L, Fwind_I);
% Xwind_e = sym('Xwind_e', [3, 1], 'real');
% Mwind_e = cross(Xwind_e, Fwind_e);
% 
% u = [Fthrust_e; Fwind_I];

Fmg_I = [0; 0; -m*g];
Fthrust_I = quatrotate_sym(quatconj_sym(L), Fthrust_e);

% 
% E_Fi_I = Fthrust_I + Fmg_I + Fwind_I;
% E_Mi_e = Mthrust_e + Mwind_e;
% 
% dR_I = V_I;
% dV_I = E_Fi_I/m;
% 
% dw_e = J_e_inv*(E_Mi_e - cross(w_e, (J_e*w_e)));
% dL = (1/2)*quatmultiply_sym(L, [0; w_e]);


eq1 = Mthrust_e == cross(Xthrust_e, Fthrust_e)
eq2 = Fx == dot(Xthrust_e, Fthrust_e)/norm(Xthrust_e)


res = solve(eq2, Fthrust_e)

eq1 = subs(eq1, Fthrust_e, [res.Fthrust_e1; res.Fthrust_e2; res.Fthrust_e3])

return

%%


X = sym('X', [13, 1], 'real');
u = sym('u', [3, 1], 'real');
p = sym('p', [25, 1], 'real');

[XXX, ~] = get_simulate(@get_symgen_step_6dof, X, u,  dt, p);

% [A, b] = equationsToMatrix(dX, X)



%%

m = 5;
dt = 1;
% 
% a = f/m;
% v = v + a*dt;
% x = x + v*dt + (a*dt^2)/2;
% 
% a   0    0   0   a   
% v = dt   1   0 * v + 
% x   dt2  dt  1   x

params.x_0 = [0; 0; -5];

params.A = [...
    0, 0, 0; ...
    dt, 1, 0; ...
    dt^2, dt, 1];

params.B = [1/m; 0; 0];
params.Q_final = diag([10, 10, 10]);

params.u_max = 2.3;

tic
[vars, status] = csolve(params);
toc


% %
% 
% figure(1)
% cla
% hold on
% 
% X = [params.x_0, cell2mat(vars.x')];
% U = [vars.u_0 cell2mat(vars.u')];
% 
% plot(0:dt:dt*(size(X, 2)-1), X(1, :), '-*', 'DisplayName', 'a');
% plot(0:dt:dt*(size(X, 2)-1), X(2, :), '-o', 'DisplayName', 'v');
% plot(0:dt:dt*(size(X, 2)-1), X(3, :), '-+', 'DisplayName', 'x');
% 
% plot(0:dt:dt*(size(U, 2)-1), U, '-d', 'DisplayName', 'u');
% 
% legend show
% grid on
