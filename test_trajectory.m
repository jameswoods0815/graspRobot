%% Mile Stone 2
Tse_initial = [0 0 1 0;  0 1 0 0;  -1 0 0 0.5; 0 0 0 1];               
Tsc_initial=[1,0,0,1;0,1,0,0;0,0,1,0;0,0,0,1];
% cube's dst configuration
thetaCube = -pi/2;
omega_cube = [0 0 1]';
v_cube = [0 -1 0]';
Tsc_dst = RpToTrans(MatrixExp3(thetaCube*VecToso3(omega_cube)),v_cube);

% The end-effector's configuration relative to the cube
Rce = MatrixExp3(2*pi/3*VecToso3([0 1 0]'));
Tce_getCube = [Rce, [0.012 0 0.003]'; 0 0 0 1];
Tce_stand_off = [Rce, [-0.06 0 0.1]'; 0 0 0 1];
k = 1;
T=k*[600 10 68 25 1000 10 68 40];
result = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_dst, Tce_getCube, Tce_stand_off, k,T);
csvwrite('step2.csv', result);