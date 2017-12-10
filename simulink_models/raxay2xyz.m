function V2 = raxay2xyz(r, ax, ay)

V = [0; 0; 1];

RX = @(a) [...
    1,      0,          0; ...
    0,      cos(a),     -sin(a);...
    0,      sin(a),     cos(a)];

RY = @(a) [...
    cos(a),     0,      sin(a); ...
    0,          1,      0;...
    -sin(a),	0,      cos(a)];

R = RY(ay) * RX(ax);

V2 = R * V;

V2 = V2 * r;

end
