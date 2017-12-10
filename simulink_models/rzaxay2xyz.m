function V = rzaxay2xyz(rz, ax, ay)

V1 = raxay2xyz(1, ax, ay);

r = rz/V1(3);

V = raxay2xyz(r, ax, ay);

end
