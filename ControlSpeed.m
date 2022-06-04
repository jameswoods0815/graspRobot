
function [ud, Jerror, Xerr] = ControlSpeed(state, H0, Tse, Tse_d, Tse_dnext, Kp, Ki, delta_t)
    % default parameters
    Blist = [0 0 1 0 0.033 0; 0 -1 0 -0.5076 0 0; 0 -1 0 -0.3526 0 0; 0 -1 0 -0.2176 0 0; 0 0 1 0 0 0]'; % body twists of the arm
    M0e = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1]; % M matrix of the arm's zero configuration
    Tb0 = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1]; % Translation of 0 from b frame 
    
    % get Jerror of arm
    Jerror_arm = JacobianBody(Blist, state(4:8)); 
    T0e = FKinBody(M0e, Blist, state(4:8));
    F = pinv(H0,1e-5);
    % get F6 
    F6 = [zeros(2,4);F;zeros(1,4)];
    % get car Jerror body;
    Jerror_car = Adjoint(TransInv(T0e)*TransInv(Tb0))*F6;
    Jerror = [Jerror_car Jerror_arm];
    % get the Ve and error; error is for storage;
    [Ve, Xerr] = FeedbackControl(Tse, Tse_d, Tse_dnext, Kp, Ki, delta_t);
    % get the control u;
    ud = pinv(Jerror,1e-5)*Ve; 
end
