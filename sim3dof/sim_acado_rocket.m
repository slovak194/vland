%% PARAMETERS SIMULATION
X0 = [-9, 19, pi/2, 0, -5, 0];
Xref = [0 a pi/2 0 0 0];
input.x = repmat(Xref, N+1, 1);
Xref = repmat(Xref, N, 1);
input.od = [];

Uref = zeros(N, n_u);
input.u = Uref;

input.y = [Xref(1:N,:) Uref];
input.yN = Xref(N,:);

input.W = diag([100 1 1 1 1 1 1 1].*10^-2);
input.WN = diag([1 1 1 1 1 1].*10^3);

iter = 0; time = 0;
Tf = 10;
KKT_MPC = []; INFO_MPC = [];
controls_MPC = [];
state_sim = X0;

figure('units','normalized','outerposition',[0 0 1 1])
% figure(1)
cla
hold on
grid on
axis equal
sk = 10;
xlim([-sk, sk])
ylim([-2*a, 2*sk])

track = plot(0,0,'.');

rocket_x = @(XX, UU) [XX(1) - cos(XX(3))*a, XX(1) + cos(XX(3))*a];
rocket_y = @(XX, UU) [XX(2) - sin(XX(3))*a, XX(2) + sin(XX(3))*a];

thurst_x = @(XX, UU) [XX(1) - cos(XX(3))*a, XX(1) - cos(XX(3))*a + cos(XX(3) + UU(2))*((2*a*UU(1))/(m*g))];
thurst_y = @(XX, UU) [XX(2) - sin(XX(3))*a, XX(2) - sin(XX(3))*a + sin(XX(3) + UU(2))*((2*a*UU(1))/(m*g))];

rocket = plot(rocket_x(state_sim, [0, 0]), rocket_y(state_sim, [0, 0]), 'LineWidth', 2);
thrust = plot(thurst_x(state_sim, [0, 0]), thurst_y(state_sim, [0, 0]));
rocketcg = plot(state_sim(1), state_sim(2), '*');

v = VideoWriter('soft_landing_01');
open(v);

while time(end) < Tf
    tic
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

    iter = iter+1;
    nextTime = iter*dt;
    disp(['current time: ' num2str(nextTime) '   ' char(9) ' (RTI step: ' num2str(output.info.cpuTime*1e6) ' µs)'])
    time = [time nextTime];

    rocket.XData = rocket_x(states.value, sim_input.u);
    rocket.YData = rocket_y(states.value, sim_input.u);
    rocketcg.XData = states.value(1);
    rocketcg.YData = states.value(2); 
    track.XData = [track.XData, states.value(1)];
    track.YData = [track.YData, states.value(2)];
    
    thrust.XData = thurst_x(states.value, sim_input.u);
    thrust.YData = thurst_y(states.value, sim_input.u);
    drawnow
    
    writeVideo(v,getframe(gca))
    
%     pause(abs(dt-toc));
end

close(v);
