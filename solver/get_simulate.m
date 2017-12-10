function [X, D] = get_simulate(step_handle, X, u, dt, p)
%#codegen

persistent ut

if isempty(ut)
    ut = zeros(size(u));
end

ut1 = u;

[k1, ~] = step_handle(dt,	X,				ut, p);
[k2, ~] = step_handle(dt,	X + (dt/2)*k1,	ut, p);
[k3, ~] = step_handle(dt,	X + (dt/2)*k2,	ut, p);
[k4, D] = step_handle(dt,	X + dt*k3,		ut1, p);

ut = ut1;

X = X + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

end
