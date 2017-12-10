function qout = quatrotate_sym(q, r)

dcm = quat2dcm_sym(q);
qout = dcm*r;

end
