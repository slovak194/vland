clear functions
clear
clc

init_params_6dof;

%%

Fthrust_e = [0; 0; m*g + 1];

Fwind_I = [0; 0; 0];
u = [Fthrust_e; Fwind_I];

R_I = [0; 0; 0];
V_I = [0; 0; 0];
w_e = [0; 0; 0];

L = dcm2quat([...
    1, 0, 0; ...
    0, 1, 0; ...
    0, 0, 1; ...
    ])';

X = [...
    R_I; ...
    V_I; ...
    w_e; ...
    L; ...
    ];

% return

%%
hf = figure(1);
set(gcf,'renderer','zbuffer')
cla

ax = gca;
h = hgtransform('Parent', ax);
grid on
axis equal
hold on

% g = 10

qf = plot3(h, ...
    [0, Fthrust_e(1)], ...
    [0, Fthrust_e(2)], ...
    [-a, -a + Fthrust_e(3)], 'c.-');

body = plot3(h, [0,0], [0,0], [-a,b], 'k', 'LineWidth', 2);

qx = plot3(h, [0,1], [0,0], [0,0], 'r');
qy = plot3(h, [0,0], [0,1], [0,0], 'g');
qz = plot3(h, [0,0], [0,0], [0,1], 'b');

axis equal
sk = 3;
xlim([-sk, sk])
ylim([-sk, sk])
zlim([-1, sk])

view([-1 -1 0.5])

% view([0 -1 0]) % XZ

T = zeros(4, 4);
T(end) = 1;
% 
% fclose('all');
% v = VideoWriter('wrong_rotation2');
% open(v);

%%
t = 0;

while t < 10
    clc
    [X, Debug] = get_simulate(@get_symgen_step_6dof, X, u, dt, p);
    t = t + dt;
%     disp(t)
    
    dL = Debug(10:13);
    
    R_I = X(1:3);
    V_I = X(4:6);
    w_e = X(7:9);
    L = X(10:13);
    
    T(1:3,1:3) = quat2dcm(quatconj(L'));
    T(1:3,4) = X(1:3);
    h.Matrix = T;
    
    drawnow
%     figure(1);
%     frame = getframe(hf);
%     writeVideo(v, frame)
    
    if abs(X(1)) > 10 || X(3) < -1
        return
    end
end

% close(v);
%%

% V = T(1:3, 1);
% V1 = [1; 0; 0];
% 
% hand_conj = @(L) [L(1); -L(2:4)];
% hand_mult_L_o_M = @(L, M) [L(1)*M(1) - dot(L(2:4), M(2:4)); L(1)*M(2:4) + M(1)*L(2:4) + cross(L(2:4),M(2:4))];
% 
% Vt = hand_mult_L_o_M(hand_mult_L_o_M(L, [0; V]), hand_conj(L));
% Vres = Vt(2:4)

% hand_rotate = @(L, V) hand_mult_L_o_M(hand_mult_L_o_M(L, [0; V]), hand_conj(L));

% А произвольный вектор, связанный с этим телом должен соответствовать своему начальному образу согласно уравнению 
% 
% V1 = L o V o L~, 
% 
% где V и V1 - векторы в подвижной и неподвижной СК, соответственно; 
% L - результат интегрирования ОДУ ПУассона; 
% L~ - сопряжённый с L кватернион; 
