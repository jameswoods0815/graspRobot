
%Inputs
%? The current actual end-effector configuration X (aka Tse)
%? The current reference end-effector configuration Xd (aka Tse,d)
%? The reference end-effector configuration at the next timestep, Xd,next (aka Tse,d,next)
%? The PI gain matrices Kp and Ki
%? The timestep ?t between reference trajectory configurations

%Outputs
%? The commanded end-effector twist V expressed in the end-effector frame {e}.

function [Ve,error] = FeedbackControl(Tse, Tse_d, Tse_dnext, Kp, Ki, delta_t)
    X = Tse;
    Xd = Tse_d;
    Xd_next = Tse_dnext;
    
    % get Vd;
    Vd_se3mat = (1/delta_t)*MatrixLog6(TransInv(Xd)*Xd_next);
    Vd = se3ToVec(Vd_se3mat);
    tmp = Adjoint(TransInv(X)*Xd)*Vd;
    %get error:
    error = se3ToVec(MatrixLog6(TransInv(X)*Xd));
    
    % This one is feedforward + PI controlller.
    Ve = tmp + Kp*error + Ki*error*delta_t;
    
    % This one is PI conttroller.
    %Ve = Kp*error + Ki*error*delta_t;
    
end

