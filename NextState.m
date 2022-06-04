%% Mile Stone 1 - youBot Kinematics Simulator


function nextState = NextState(state, controlInput, delta_t, speed_limit, H)
    
% limit the speed;
    for i = 1:length(controlInput)
        
        if controlInput(i) > speed_limit
            controlInput(i) = speed_limit;
        end
        
        if controlInput(i) < -speed_limit
            controlInput(i) = -speed_limit;
        end
    end 
    
    %new arm joint angles = (old arm joint angles) + (joint speeds)?t

    %new wheel angles = (old wheel angles) + (wheel speeds)?t

    %new chassis configuration is obtained from odometry
 
    nextState = state;
    nextState(4:12) = state(4:12) + controlInput*delta_t;
    nextState(1:3) = nextState(1:3) + pinv(H,1e-4)*controlInput(6:9)*delta_t;
end