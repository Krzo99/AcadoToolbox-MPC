function ACADO_main(N, dt)

cd ./ACADO/

numIntegratorSteps = 2;

% 1: qpOASES 2: qpDUNES
solver_selection = 1;

DifferentialState x y yaw velocity steer acceleration; 
Control del_steer del_acceleration; % input rate (-> per sec)

OnlineData cylinder1_x
OnlineData cylinder1_y
OnlineData cylinder1_r

OnlineData cylinder2_x
OnlineData cylinder2_y
OnlineData cylinder2_r

OnlineData cylinder3_x
OnlineData cylinder3_y
OnlineData cylinder3_r

OnlineData cylinder4_x
OnlineData cylinder4_y
OnlineData cylinder4_r

OnlineData cylinder5_x
OnlineData cylinder5_y
OnlineData cylinder5_r


f = dot([x; y; yaw; velocity; steer; acceleration]) == ...
    [velocity*cos(yaw);
     velocity*sin(yaw);
     tan(steer);
     acceleration;
     del_steer;
     del_acceleration
    ];

% Ubistvu je to cost function obsticla
obs1 = 10/(1+exp(10*(sqrt((x-cylinder1_x)^2  + (y-cylinder1_y)^2 ) - 0.5 - cylinder1_r)));
obs2 = 10/(1+exp(10*(sqrt((x-cylinder2_x)^2  + (y-cylinder2_y)^2 ) - 0.5 - cylinder2_r)));
obs3 = 10/(1+exp(10*(sqrt((x-cylinder3_x)^2  + (y-cylinder3_y)^2 ) - 0.5 - cylinder3_r)));
obs4 = 10/(1+exp(10*(sqrt((x-cylinder4_x)^2  + (y-cylinder4_y)^2 ) - 0.5 - cylinder4_r)));
obs5 = 10/(1+exp(10*(sqrt((x-cylinder5_x)^2  + (y-cylinder5_y)^2 ) - 0.5 - cylinder5_r)));

obs = obs1+...
obs2+...
obs3+...
obs4+...
obs5;


%% SIMexport (Integrator Setting)
acadoSet('problemname', 'sim');
sim = acado.SIMexport( dt );
sim.setModel(f);
sim.set( 'INTEGRATOR_TYPE',             'INT_EX_EULER' );
sim.set( 'NUM_INTEGRATOR_STEPS',         numIntegratorSteps );

integrate_name = './../integrate_robot';
ACADO_name = './../ACADO_robot';
integrate_name = sprintf(integrate_name, N*dt, N);
ACADO_name = sprintf(ACADO_name, N*dt, N);

sim.exportCode( 'export_SIM' );
cd export_SIM
make_acado_integrator(integrate_name);
cd ..

%% MPCexport (optimal control setting)
acadoSet('problemname', 'NMPC');

n_DiffStates = length(diffStates);
n_Controls = length(controls);

h = [diffStates; controls; obs];
hN = diffStates;

ocp = acado.OCP( 0.0, dt*N, N );
W_mat = eye(n_DiffStates+n_Controls+1,n_DiffStates+n_Controls+1);
WN_mat = eye(n_DiffStates,n_DiffStates);
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);
ocp.minimizeLSQ( W, h );
ocp.minimizeLSQEndTerm( WN, hN );

ocp.subjectTo( -0.4363 <= steer <= 0.4363 );
ocp.subjectTo( -5 <= acceleration   <= 3 );
ocp.subjectTo( 0 <= velocity   <= 5 );

ocp.subjectTo( -0.4363 <= del_steer <= 0.4363 );
ocp.subjectTo( -5 <= del_acceleration   <= 3 );

ocp.subjectTo( sqrt((x-cylinder1_x)*(x-cylinder1_x)  + (y-cylinder1_y)*(y-cylinder1_y)) - cylinder1_r + 0.5 >= 0 )
ocp.subjectTo( sqrt((x-cylinder2_x)*(x-cylinder2_x)  + (y-cylinder2_y)*(y-cylinder2_y)) - cylinder2_r + 0.5 >= 0 )
ocp.subjectTo( sqrt((x-cylinder3_x)*(x-cylinder3_x)  + (y-cylinder3_y)*(y-cylinder3_y)) - cylinder3_r + 0.5 >= 0 )
ocp.subjectTo( sqrt((x-cylinder4_x)*(x-cylinder4_x)  + (y-cylinder4_y)*(y-cylinder4_y)) - cylinder4_r + 0.5 >= 0 )
ocp.subjectTo( sqrt((x-cylinder5_x)*(x-cylinder5_x)  + (y-cylinder5_y)*(y-cylinder5_y)) - cylinder5_r + 0.5 >= 0 )
ocp.setModel(f);

mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'PRINTLEVEL',                  'NONE'              );
mpc.set( 'INTEGRATOR_TYPE',             'INT_EX_EULER'      );
mpc.set( 'CG_USE_OPENMP ',              'YES');
mpc.set( 'NUM_INTEGRATOR_STEPS',         2*N                );
mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-4				);

switch solver_selection
    case 1
        disp('qpOASES selected!');
        % qpOASES dense solver
        mpc.set( 'QP_SOLVER',           'QP_QPOASES'        );
        mpc.set( 'SPARSE_QP_SOLUTION',  'FULL_CONDENSING_N2');
        mpc.set( 'HOTSTART_QP',         'YES'             	);
    case 2
        disp('qpDUNES selected!');
        % qpDUNES sparse solver
        mpc.set( 'SPARSE_QP_SOLUTION',  'SPARSE_SOLVER'     );
        mpc.set( 'QP_SOLVER',           'QP_QPDUNES'        );
    otherwise
        disp('Please select a solver!');
end

mpc.exportCode( 'export_MPC' );
switch solver_selection
    case 1
        disp('qpOASES exported!');
        copyfile('/home/domen/Documents/ACADOtoolkit/external_packages/qpoases','export_MPC/qpoases', 'f')
    case 2
        disp('QP_QPDUNES exported!');
        copyfile('/home/domen/Documents/ACADOtoolkit/external_packages/qpdunes', 'export_MPC/qpdunes', 'f')
    otherwise
        disp('Please select a solver!');
end
cd export_MPC
    make_acado_solver(ACADO_name)
cd ..
cd ..