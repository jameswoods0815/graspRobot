File:
Function:  Control Speed: control every motor speed; -need feedbackControl
Function:  FeedbackControl: feedback contorl function
Function:  NextState
Function: SampleTime, time interplation fucntion:
Function: TracjectoryGenerator

Test:
mile1test: test mile stone1
test_tracjectory.m test tracjectory generaction
mae_project: commbine all the funtion together:

For detail Test:
open the mae_project;
change the parameter:

Kp = 1*eye(6); % P paramenter
Ki = 0.2*eye(6); % I parameter
Tse_initial = [0 0 1 0;  0 1 0 0;  -1 0 0 0.5; 0 0 0 1];  % end-effector position     
Tsc_initial=[1,0,0,0.9;0,1,0,0;0,0,1,0;0,0,0,1]; % cube init position
% cube's dst configuration
thetaCube = -pi/2;
omega_cube = [0 0 1]';
v_cube = [0 -0.9 0]';
Tsc_dst = RpToTrans(MatrixExp3(thetaCube*VecToso3(omega_cube)),v_cube);% cube destination position
k = 2; % this k is for tracjectory generation

This is the funciton you can change, don't change the other code
