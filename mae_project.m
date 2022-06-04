%If you want to test the algorithm:
% You need to change:  Tse_initial Tsc_initial Tsc_dst;for yor own
% cofiguration;

% And you should check the distance to set different K parameter to
% generate different tracjectory;

% If you want to test the error: you need to change the Init error;
 
% And you should change the KP ,KI parameter;

clear all;
clc;
close all
addpath('mr')

% %%%%%%%%%%%%%%%%%%%%%%%%%%New task Parameter%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Kp = 0.03*eye(6); % P paramenter
Ki = 0.01*eye(6); % I parameter
Tse_initial = [0 0 1 0;  0 1 0 0;  -1 0 0 0.5; 0 0 0 1];  % end-effector position     
Tsc_initial=[1,0,0,0.9;0,1,0,0;0,0,1,0;0,0,0,1]; % cube init position
% cube's dst configuration
thetaCube = -pi/2;
omega_cube = [0 0 1]';
v_cube = [0 -0.9 0]';
Tsc_dst = RpToTrans(MatrixExp3(thetaCube*VecToso3(omega_cube)),v_cube);% cube destination position
k = 1; % this k is for tracjectory generation
T=k*[300 10 68 25 1000 10 68 40];
%%%%%%%%%%%%%%%%%%%%%%%%New Task prameter%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%% un comment over Shoot parameter%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kp = 1*eye(6); % P paramenter
% Ki = 0.2*eye(6); % I parameter
% Tse_initial = [0 0 1 0;  0 1 0 0;  -1 0 0 0.5; 0 0 0 1];  % end-effector position     
% Tsc_initial=[1,0,0,1;0,1,0,0;0,0,1,0;0,0,0,1]; % cube init position
% % cube's dst configuration
% thetaCube = -pi/2;
% omega_cube = [0 0 1]';
% v_cube = [0 -1 0]';
% Tsc_dst = RpToTrans(MatrixExp3(thetaCube*VecToso3(omega_cube)),v_cube);% cube destination position
% k = 1; % this k is for tracjectory generation
% %%set the step
% T=k*[300 10 68 25 1000 10 68 40];
%%%%%%%%%%%%%%%%%%%%%%%%% uncommnet to run over shoot%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% %%%%%%%%%%%%%%%%%%%%%%%%%%Uncomment this to run best%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kp = 0.05*eye(6); % P paramenter
% Ki = 0.005*eye(6); % I parameter
% Tse_initial = [0 0 1 0;  0 1 0 0;  -1 0 0 0.5; 0 0 0 1];  % end-effector position     
% Tsc_initial=[1,0,0,1;0,1,0,0;0,0,1,0;0,0,0,1]; % cube init position
% %cube's dst configuration
% thetaCube = -pi/2;
% omega_cube = [0 0 1]';
% v_cube = [0 -1 0]';
% Tsc_dst = RpToTrans(MatrixExp3(thetaCube*VecToso3(omega_cube)),v_cube);% cube destination position
% k = 1; % this k is for tracjectory generation
% % set the step
% T=k*[600 10 68 25 1000 10 68 40];
% % % %%%%%%%%%%%%%%%%%%%%%%%%%Uncomment this to run best%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Mile Stone 1 - youBot Kinematics Simulator
%init state
%r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz, gripper state
state_initial = [0 0 0 0 0 0.2 -1.6 0 0 0 0 0 0]';
% body cofiguration 
testNow=state_initial;
%(9 variables: 5 for arm ? ?, 4 for wheels u)
control_u0 =[0 0 0 0 0 0 0 0 0]';
delta_t = 1;
speed_limit = 20;
l = 0.47/2;
w = 0.3/2;
r = 0.0475;
% H matrix for car
H = (1/r)*[-l-w  1 -1; l+w  1  1; l+w  1 -1; -l-w  1  1];
tmpResult=zeros(100,13);
for i=1:100
  tmp=NextState(testNow, [0.05; 0.05 ;0.05; 0.05; 0.05; 15; 15; 15; 15;], delta_t, speed_limit, H);
  tmpResult(i,:)=tmp;
  testNow=tmp;
end
csvwrite('nextState',tmpResult);

%% Mile Stone 2

% The end-effector's configuration relative to the cube
Rce = MatrixExp3(2*pi/3*VecToso3([0 1 0]'));
Tce_getCube = [Rce, [0.012 0 0.003]'; 0 0 0 1];
Tce_stand_off = [Rce, [-0.06 0 0.1]'; 0 0 0 1];
result = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_dst, Tce_getCube, Tce_stand_off, k,T);
csvwrite('step2.csv', result);

%% Final result
delta_t = 1; 
%set speed limint:
speed_limit = 20;
Blist = [0 0 1 0 0.033 0; 0 -1 0 -0.5076 0 0; 0 -1 0 -0.3526 0 0; 0 -1 0 -0.2176 0 0; 0 0 1 0 0 0]'; 
M0e = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1]; % M matrix 
Tb0 = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1];
Tse = Tse_initial; % end effector initial configuration
% add error for Tse:
R60 = [1 0 0; 0 cos(pi/6) -sin(pi/6); 0 sin(pi/6) cos(pi/6)];
Terror = [R60 [0.2; 0; 0.0]; 0 0 0 1];
Tse=Tse*Terror;
state = state_initial; % angle inital state
angelList = []; % list to store all the joints angles
errorList = []; % list to store all the errors
for i = 1: length(result)-1
    % get a state from the config list
    nowState = result(i,:);
    %get Tse matrix
    Tse_d = [nowState(1:3) nowState(10); nowState(4:6) nowState(11); nowState(7:9) nowState(12); 0 0 0 1];
    nextState = result(i+1,:);
    %get next state matrix
    Tse_dnext = [nextState(1:3) nextState(10); nextState(4:6) nextState(11); nextState(7:9) nextState(12); 0 0 0 1];
    % get the Joint speed, Jocobian error and error for every angel;
    [ud, Jerr,Xerr] = ControlSpeed(state, H, Tse, Tse_d, Tse_dnext, Kp, Ki, delta_t);
    errorList = [errorList; Xerr'];
    u = [ud(5:9); ud(1:4)];  
    % get to the next state
    state = NextState(state, u, delta_t, speed_limit, H);
    angelList = [angelList; [state(1:12)' nowState(13)]];
    % get the body angel;
    phi = state(1);
    x = state(2);
    y = state(3);
    %ge Tsb 
    Tsb = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 0.0963; 0 0 0 1];
    T0e = FKinBody(M0e, Blist, state(4:8));
    % get Tse
    Tse = Tsb*Tb0*T0e;
end
%write the error and angel list to animation

plot(errorList)
xlabel('step');
ylabel('error')
legend('angel1','angel2','angel3','angel4','angel5','angel6')
title('new- Pk=0.03,pi=0.01')
csvwrite('over.csv', angelList);
csvwrite('errorover.csv',errorList);