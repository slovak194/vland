clc;
clear all;

%% Parameters
m = 1;
a = 1;
I = 1;
g = 9.80665;

dt = 0.1;

problem = get_problem();

DifferentialState(problem.X{:});
Control(problem.u{:});

n_X = length(diffStates);
n_u = length(controls);

X = [x; y; psi; x_dot; y_dot; psi_dot];
u = [F; theta];
p = [I; a; g; m];

EXPORT = 1;
acado_root = '/home/slovak/vertical_landing/ACADOtoolkit';

%% Differential Equation
f = dot(X) == is(get_symgen_step(dt,X,u,p));

h = [diffStates; controls];
hN = [diffStates];

%% SIMexport
acadoSet('problemname', 'sim');

numSteps = 5;
sim = acado.SIMexport(dt);
sim.setModel(f);
sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_RIIA5' );
sim.set( 'NUM_INTEGRATOR_STEPS',        numSteps        );

if EXPORT
    sim.exportCode( 'export_SIM' );
    
    cd export_SIM
    make_acado_integrator('../integrate_rocket')
    cd ..
end

%% MPCexport
acadoSet('problemname', 'mpc');

N = 20;
ocp = acado.OCP(0.0, N*dt, N);

W_mat = eye(n_X+n_u, n_X+n_u);
WN_mat = eye(n_X, n_X);
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);

ocp.minimizeLSQ( W, h );
ocp.minimizeLSQEndTerm( WN, hN );

Slx = [0 0 0 0 0 0];
Slu = [0 0];
ocp.minimizeLSQLinearTerms(Slx, Slu);
% 

ocp.subjectTo( 0 <= F <= 1.5*m*g);
ocp.subjectTo( -2*pi/180 <= theta <= 2*pi/180);

ocp.subjectTo( -10 <= x <= 10);
ocp.subjectTo( 0 <= y <= 20);
ocp.subjectTo( (pi/2+pi/6) <= psi <= (pi/2-pi/6));

ocp.subjectTo( -3 <= y_dot <= 3);

% ocp.subjectTo( -10.0 <= [uC; uL] <= 10.0 );
% ocp.subjectTo( -100.0 <= [duC;duL] <= 100.0 );
% ocp.subjectTo( -0.3 <= vC <= 0.3 );
% ocp.setLinearInput(M1,A1,B1);
ocp.setModel(f);
%
mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',        2*N                 );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
mpc.set( 'HOTSTART_QP',                 'NO'             	);
mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-10				);
% mpc.set( 'GENERATE_SIMULINK_INTERFACE', 'YES'               );

if EXPORT
    mpc.exportCode( 'export_MPC' );
    copyfile([acado_root, '/external_packages/qpoases'], 'export_MPC/qpoases', 'f')

    cd export_MPC
    make_acado_solver('../acado_MPCstep')
    cd ..
end

rehash

sim_acado_rocket;