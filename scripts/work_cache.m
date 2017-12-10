clc

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
