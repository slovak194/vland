yaw = 0.7854;
pitch = 0.1;
roll = -0.3;
q = angle2quat( yaw, pitch, roll );

v = [1, 0, 0];

v2 = quatrotate_sym(q', v');
v3 = quatrotate(q, v)';

assert(all(abs(v2 - v3) < 10^-14))

yaw = 0.7;
pitch = -0.1;
roll = 0.4;
q2 = angle2quat( yaw, pitch, roll );


q3 = quatmultiply_sym(q, q2);
q4 = quatmultiply(q, q2)';

assert(all(abs(q3 - q4) < 10^-14))

%%
clc

vec = [1, 0, 0];

axang = [0 0 1 1];
quat = axang2quat(axang)

quatrotate(quatconj(quat), V')

% 0.9659         0   -0.2588         0

% vec2 = quatrotate_sym(quat', vec')

%%

v1 = [0;0;1];
v2 = [1;-1;-1]

abs(dot(v1,v2)/(norm(v1)*norm(v2)))


cos(pi/6)