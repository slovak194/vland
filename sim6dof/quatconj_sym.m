function qout = quatconj_sym( qin ) 

qout = qin;

qout(1) = qin(1);
qout(2) = -qin(2);
qout(3) = -qin(3);
qout(4) = -qin(4);
