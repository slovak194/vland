g = 9.80665 * 0;

rocket_heigth = 2;
rocket_radius = 0.05;

rocket_density = 1000;
rocket_weight = rocket_density*rocket_heigth*pi*(rocket_radius^2);



return

%%

clc

r = 2;

V = raxay2xyz(1, 13*pi/180, 13*pi/180)

r = r/V(3);

V = raxay2xyz(r, 13*pi/180, 0)

sum(V.^2)

%%

V = rzaxay2xyz(2, 13*pi/180, 13*pi/180)

%%

alpha, betta, rz

R

R = rx/cos(alpha)
R = ry/cos(betta)
R = rz/cos(gamma)

cos(alpha)^2 + cos(betta) + cos(gamma)^2 = 1

cos(gamma)^2 = 1 - cos(alpha)^2 - cos(betta)^2

R^2 = 

rx = cos(alpha) * R;
ry = cos(betta) * R;
rz = cos(gamma) * R;

rz = sqrt(R^2 - rx^2 - ry^2);


V = [rx, ry, rz];