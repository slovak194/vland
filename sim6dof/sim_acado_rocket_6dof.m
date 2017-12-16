clc
clear
init_params_6dof;

X0 = [...
    -2;  0;  2; ...
    0;  0;  0; ...
    0;  0;  0; ...
    1;  0;  0;  0; ...
    ]';

Xref = [...
    0;  0;  a; ...
    0;  0;  0; ...
    0;  0;  0; ...
    1;  0;  0;  0; ...
    ]';

input.W = diag([...
    1 1 1 ...
    1 1 1 ...
    1 1 1 ...
    1 1 1 1 ...
    1 1 1 ...
    ].*10^0);
input.WN = diag([...
    100 100 1 ...
    10 10 50 ...
    1 1 1 ...
    1 1 1 1 ...
    ].*10^4);

%%

input.x = repmat(Xref, N+1, 1);
Xref = repmat(Xref, N, 1);
% input.od = repmat(p', N+1, 1);
input.od = [];

Uref = zeros(N, n_u);
input.u = Uref;

input.y = [Xref(1:N,:) Uref];
input.yN = Xref(N,:);

iter = 0; time = 0;
Tf = 100;
KKT_MPC = []; INFO_MPC = [];
controls_MPC = [];
state_sim = X0;
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

plot3(Xref(1,1), Xref(1,2), Xref(1,3), '*')
plot3(X0(1,1), X0(1,2), X0(1,3), 'o')

track = plot3(0,0,0,'.');

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

while time(end) < inf
    i = i+1;
    % Solve NMPC OCP
    input.x0 = state_sim(end,:);
    output = acado_MPCstep(input);
    % Save the MPC step
    INFO_MPC = [INFO_MPC; output.info];
    KKT_MPC = [KKT_MPC; output.info.kktValue];
    controls_MPC = [controls_MPC; output.u(1,:)];
    input.x = output.x;
    input.u = output.u;
    
    % Simulate system
    sim_input.x = state_sim(end,:).';
    sim_input.u = output.u(1,:).';
    states = integrate_rocket(sim_input);
    state_sim = [state_sim; states.value'];
    
    u = [u, sim_input.u];
    
    iter = iter+1;
    nextTime = iter*dt;
    disp(['current time: ' num2str(nextTime) '   ' char(9) ' (RTI step: ' ...
        num2str(100*output.info.cpuTime/dt) ' %)'])
    time = [time nextTime];
    
    qf.XData = [0, 2*a*sim_input.u(1)/(m*g)];
    qf.YData = [0, 2*a*sim_input.u(2)/(m*g)];
    qf.ZData = [-a, -a + 2*a*sim_input.u(3)/(m*g)];
    
    R_I = states.value(1:3);
    V_I = states.value(4:6);
    w_e = states.value(7:9);
    L = states.value(10:13);
    
    [R_I, V_I, w_e, sim_input.u];
    
    T(1:3,1:3) = quat2dcm(quatconj(L'));
    T(1:3,4) = R_I;
    h.Matrix = T;
    
    track.XData = [track.XData, states.value(1)];
    track.YData = [track.YData, states.value(2)];
    track.ZData = [track.ZData, states.value(3)];
    
    drawnow
    
    if R_I(3) < a || ~ishandle(hf2)
        break
    end
end

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
