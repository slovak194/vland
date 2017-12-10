function qinv = quatinv_sym( q )

qinv  = quatconj_sym( q )./(sum(q.*q));
