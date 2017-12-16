clc
clear
clear all
clear functions
init_params_6dof;

X0 = [...
    -3;  0;  1; ...
    0;  0;  0; ...
    0;  0;  0; ...
    1;  0;  0;  0; ...
    ]';

Xref = [...
    -2.5;  0;  2; ...
    0;  0;  0; ...
    0;  0;  0; ...
    1;  0;  0;  0; ...
    ]';

%%

mpc_input.W = diag([...
    1 1 1 ...
    1 1 1 ...
    1 1 1 ...
    1 1 1 1 ...
    1 1 1 ...
    ].*10^0);

mpc_input.WN = diag([...
    100 100 100 ...
    10 10 50 ...
    1 1 1 ...
    0 0 0 0 ...
    ].*10^4);

Xref_grid = repmat(Xref, N+1, 1);
mpc_input.x = Xref_grid;
mpc_input.od = repmat([0; 0; 0]', N+1, 1);
mpc_input.u = zeros(N, n_u);
mpc_input.y = [Xref_grid(1:end-1, :) zeros(N, n_u)];
mpc_input.yN = Xref;

iter = 0; time = 0;
Tf = 100;
KKT_MPC = []; INFO_MPC = [];
controls_MPC = {};
state_sim = X0;

X.value = X0';

u = [0;0;0];

%%
hf2 = figure(2);
hf = figure(1);
set(gcf,'renderer','zbuffer')
cla

ax = gca;
h = hgtransform('Parent', ax);
grid on
% axis equal
hold on

Xrefplot = plot3(Xref(1,1), Xref(1,2), Xref(1,3), '*');
plot3(X0(1,1), X0(1,2), X0(1,3), 'o')

track = plot3(0,0,0,'.');
predicted_track = plot3(0,0,0,'.r');

qf = plot3(h, ...
    [0, 1], ...
    [0, 1], ...
    [-a, -a + 1], 'c.-');

body = plot3(h, [0,0], [0,0], [-a,b], 'k', 'LineWidth', 2);

qx = plot3(h, [0,1], [0,0], [0,0], 'r');
qy = plot3(h, [0,0], [0,1], [0,0], 'g');
qz = plot3(h, [0,0], [0,0], [0,1], 'b');

sk = 1;

min_x = min(X0(1), Xref(1,1)) - sk;
max_x = max(X0(1), Xref(1,1)) + sk;

min_y = min(X0(2), Xref(1,2)) - sk;
max_y = max(X0(2), Xref(1,2)) + sk;

max_z = max(X0(3), Xref(1,3)) + sk*4;

xlim([min_x, max_x])
ylim([min_y, max_y])
zlim([-1, max_z])

% view([-1 -1 0.5])
view([0 -1 0]) % XZ

T = zeros(4, 4);
T(end) = 1;

R_I = X0(1:3);
V_I = X0(4:6);
w_e = X0(7:9);
L = X0(10:13);

T(1:3,1:3) = quat2dcm(L);
T(1:3,4) = R_I;
h.Matrix = T;

drawnow
%%

stop = false;
figure(2);

while time(end) < inf
    % Solve NMPC OCP
    mpc_input.x0 = X.value';
    mpc_output = mpc_6dof_step(mpc_input);
    
    
    % Simulate system
    sim_input.x = X.value;
    sim_input.od = mpc_input.od(1,:)';
    sim_input.u = mpc_output.u(1,:).';
    X = sim_6dof_step(sim_input);
    
    k_norm = norm(X.value - Xref');
    
    if false %k_norm < 0.3
        Xref(1) = Xref(1) + 0.3;
        Xref_grid = repmat(Xref, N+1, 1);
        mpc_input.x = Xref_grid;
        mpc_input.od = repmat([0; 0; 0]', N+1, 1);
        mpc_input.u = zeros(N, n_u);
        mpc_input.y = [Xref_grid(1:end-1, :) zeros(N, n_u)];
        mpc_input.yN = Xref;
    else
        mpc_input.x = mpc_output.x;
        mpc_input.u = mpc_output.u;
    end
    
    
    
    
    % Save values
    INFO_MPC = [INFO_MPC; mpc_output.info];
    KKT_MPC = [KKT_MPC; mpc_output.info.kktValue];
    controls_MPC = [controls_MPC; mpc_output.u];
    state_sim = [state_sim; X.value'];
    u = [u, sim_input.u];
    
    % Visualize
    iter = iter+1;
    nextTime = iter*dt;
    time = [time nextTime];
    clc
    disp(['time: ', num2str(nextTime), char(9),...
        '(RT: ', num2str(100*mpc_output.info.cpuTime/dt) ' %)', char(9), ...
        'k_norm: ', num2str(k_norm)]);
    
    
    qf.XData = [0, 2*a*sim_input.u(1)/(m*g)];
    qf.YData = [0, 2*a*sim_input.u(2)/(m*g)];
    qf.ZData = [-a, -a + 2*a*sim_input.u(3)/(m*g)];
    
    R_I = X.value(1:3);
    V_I = X.value(4:6);
    w_e = X.value(7:9);
    L = X.value(10:13);
    
    T(1:3,1:3) = quat2dcm(quatconj(L'));
    T(1:3,4) = R_I;
    h.Matrix = T;
        
    Xrefplot.XData = Xref(1,1);
    Xrefplot.YData = Xref(1,2);
    Xrefplot.ZData = Xref(1,3);
        
    track.XData = [track.XData, X.value(1)];
    track.YData = [track.YData, X.value(2)];
    track.ZData = [track.ZData, X.value(3)];
    
    predicted_track.XData = mpc_output.x(:, 1);
    predicted_track.YData = mpc_output.x(:, 2);
    predicted_track.ZData = mpc_output.x(:, 3);
    
    drawnow
    
    if R_I(3) < a || ~ishandle(hf2) || k_norm < 0.06
        break
    end
end

% [X.value, input.x0', X.value-input.x0']

%%

figure(2)
cla
hold on
grid on

plot([time(1), time(end)], [lateral_thrust_lim, lateral_thrust_lim], '--', 'DisplayName', 'lateral_thrust_lim')
plot([time(1), time(end)], [longitudal_thrust_up_lim, longitudal_thrust_up_lim], '--', 'DisplayName', 'longitudal_thrust_up_lim')
plot([time(1), time(end)], [longitudal_thrust_down_lim, longitudal_thrust_down_lim], '--', 'DisplayName', 'longitudal_thrust_down_lim')

plot(time, u(1,:), '.-', 'DisplayName', 'u1')
plot(time, u(2,:), '.-', 'DisplayName', 'u2')
plot(time, u(3,:), '.-', 'DisplayName', 'u3')

legend show

figure(3)
cla
hold on
grid on

plot(time, state_sim(:,4), '.-', 'DisplayName', 'V_I1')
plot(time, state_sim(:,5), '.-', 'DisplayName', 'V_I2')
plot(time, state_sim(:,6), '.-', 'DisplayName', 'V_I3')

legend show

% close(v);
