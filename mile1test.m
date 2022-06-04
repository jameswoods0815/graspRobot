%% Mile Stone 1 - youBot Kinematics Simulator

%This function is to test the nextstate:
%init state
state_initial = [0 0 0 0 0 0.2 -1.6 0 0 0 0 0 0]';

testNow=state_initial;
% control variable;
control_u0 =[0 0 0 0 0 0 0 0 0]';
delta_t = 1;
speed_limit = 20;
l = 0.47/2;
w = 0.3/2;
r = 0.0475;
%H function
H = (1/r)*[-l-w  1 -1; l+w  1  1; l+w  1 -1; -l-w  1  1];

tmpResult=zeros(100,13);

% get every nextState; 100 step;
for i=1:100
    
  tmp=NextState(testNow, [0.1; 0.1 ;-0.1; -0.1;0.1; 0.1; 0.1; 0.1; 0.1;], delta_t, speed_limit, H);
  tmpResult(i,:)=tmp;
  testNow=tmp;
end

%write csv
csvwrite('nextState',tmpResult);