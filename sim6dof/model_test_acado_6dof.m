clear functions
clear
clear all
clc

init_params_6dof;

DifferentialState R_I(3,1)
DifferentialState V_I(3,1)
DifferentialState w_e(3,1)
DifferentialState L(4,1)

% AlgebraicState

Control Fthrust_e(3,1)

u = [Fthrust_e];

% Disturbance

X = [...
    R_I; ...
    V_I; ...
    w_e; ...
    L; ...
    ];

% Differential Equation
f = dot(X) == is(get_symgen_step_6dof(dt, X, u, p));

h = [diffStates; controls];
hN = [diffStates];

% SIMexport
acadoSet('problemname', 'sim');

numSteps = 5;
sim = acado.SIMexport(dt);
sim.setModel(f);
sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_RIIA5' );
sim.set( 'NUM_INTEGRATOR_STEPS',        numSteps        );

%%

if EXPORT
    sim.exportCode( 'export_SIM' );
    
    cd export_SIM
    make_acado_integrator('../../generated/integrate_rocket')
    cd ..
end

%% MPCexport
acadoSet('problemname', 'mpc');

ocp = acado.OCP(0.0, N*dt, N);

W_mat = eye(n_X+n_u, n_X+n_u);
WN_mat = eye(n_X, n_X);
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);

ocp.minimizeLSQ( W, h );
ocp.minimizeLSQEndTerm( WN, hN );

%%
%
ocp.subjectTo( -lateral_thrust_lim <= Fthrust_e1 <= lateral_thrust_lim);
ocp.subjectTo( -lateral_thrust_lim <= Fthrust_e2 <= lateral_thrust_lim);
ocp.subjectTo( longitudal_thrust_down_lim <= Fthrust_e3 <= longitudal_thrust_up_lim);

alpha_lim = sin((pi/12)/2);

ocp.subjectTo( -alpha_lim <= L2 <= alpha_lim);
ocp.subjectTo( -alpha_lim <= L3 <= alpha_lim);

% ocp.subjectTo( -1 <= w_e <= 1);

ocp.subjectTo( -5 <= V_I3 <= 5);

%%

ocp.setModel(f);
%
mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',        2*N                 );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
mpc.set( 'HOTSTART_QP',                 'YES'             	);
mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-10				);
% mpc.set( 'GENERATE_SIMULINK_INTERFACE', 'YES'               );

%%

if EXPORT
    mpc.exportCode( 'export_MPC' );
    copyfile([acado_root, '/external_packages/qpoases'], 'export_MPC/qpoases', 'f')
    
    cd export_MPC
    make_acado_solver('../acado_MPCstep')
%     make_acado_solver_sfunction;
    cd ..
    
    delete('./*acadodata*');
    delete('./*RUN*');
    delete('./*.cpp');
end

rehash

sim_acado_rocket_6dof;