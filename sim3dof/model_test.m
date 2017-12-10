clear functions

Ts = 0.01;

m = 1;
a = 1;
I = 1;
g = 9.80665;

% close all

x = 4;
y = 4;
psi = pi/2;
x_dot = 0;
y_dot = 0;
psi_dot = 0;

theta = 5*pi/180;
F = m*g + 0.1;
u = [F; theta];

clc
X = [...
	x; ...
	y; ...
	psi; ...
	x_dot; ...
	y_dot; ...
	psi_dot; ...
	]

figure(1)
cla
hold on
grid on
axis equal
sk = 10;
xlim([-sk, sk])
ylim([-sk, sk])

track = plot(0,0,'.');

rocket_x = @(XX, UU) [XX(1) - cos(XX(3))*a, XX(1) + cos(XX(3))*a];
rocket_y = @(XX, UU) [XX(2) - sin(XX(3))*a, XX(2) + sin(XX(3))*a];

thurst_x = @(XX, UU) [XX(1) - cos(XX(3))*a, XX(1) - cos(XX(3))*a + cos(XX(3) + UU(2))*((2*a*UU(1))/(m*g))];
thurst_y = @(XX, UU) [XX(2) - sin(XX(3))*a, XX(2) - sin(XX(3))*a + sin(XX(3) + UU(2))*((2*a*UU(1))/(m*g))];

rocket = plot(rocket_x(X, u), rocket_y(X, u));
thrust = plot(thurst_x(X, u), thurst_y(X, u));
rocketcg = plot(X(1), X(2), '*');

%%

F = m*g + 0.1;
theta = 5*pi/180;

u = [F; theta];

figure(1)

axis equal
xlim([-sk, sk])
ylim([-sk, sk])

p = [I; a; g; m];

PP{1} = p;
PP{2} = [];

for i = 1:inf
clc
X = get_simulate(@get_symgen_step, X, u, Ts, p)
disp(i)

if mod(i, 10) == 0
    rocket.XData = rocket_x(X, u);
    rocket.YData = rocket_y(X, u);
    rocketcg.XData = X(1);
    rocketcg.YData = X(2); 
    track.XData = [track.XData, X(1)];
    track.YData = [track.YData, X(2)];
    
    thrust.XData = thurst_x(X, u);
    thrust.YData = thurst_y(X, u);
    
    drawnow
end

if abs(X(1)) > 10 || X(2) < 0
    return
end

end


