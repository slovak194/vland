function problem = get_problem()
%GET_PROBLEM Summary of this function goes here
%   Detailed explanation goes here

problem.X = {'x', 'y', 'psi', 'x_dot', 'y_dot', 'psi_dot'};
problem.u = {'F', 'theta'};

problem.dd = {'x_ddot', 'y_ddot', 'psi_ddot'};
problem.lX = {'ax', 'ay', 'vx', 'vy'};

problem.p = {'I', 'a', 'dt', 'g', 'm'};

end
