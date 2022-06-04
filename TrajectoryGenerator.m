
function result = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k, T)
    result = [];
    T_se = [];
    % step 1 Move the gripper from its initial configuration to a standoff configuration a few cm above the block.
    for t = 0:T(1)
        T_se = Tse_initial*MatrixExp6(SampleTime(T(1), t)*MatrixLog6(TransInv(Tse_initial)*Tsc_initial*Tce_standoff));
        result = [result; [T_se(1,1:3), T_se(2,1:3), T_se(3,1:3), T_se(1:3,4)', 0]];
    end
    % step 2  Move the gripper down to the grasp position.
    for t = 0:T(2)
        T_se = T_se*MatrixExp6(SampleTime(T(2), t)*MatrixLog6(TransInv(T_se)*Tsc_initial*Tce_grasp));
        result = [result; [T_se(1,1:3), T_se(2,1:3), T_se(3,1:3), T_se(1:3,4)', 0]];
    end
    % step 3Close the gripper2.
    for t = 0:T(3)
        result = [result; [T_se(1,1:3), T_se(2,1:3), T_se(3,1:3), T_se(1:3,4)', 1]];
    end
    % step 4  Move the gripper back up to the ?standoff? configuration.
    for t = 0:T(4)
        T_se = T_se*MatrixExp6(SampleTime(T(4), t)*MatrixLog6(TransInv(T_se)*Tsc_initial*Tce_standoff));
        result = [result; [T_se(1,1:3), T_se(2,1:3), T_se(3,1:3), T_se(1:3,4)' ,1]];
    end
    % step 5 Move the gripper to a ?standoff? configuration above the final configuration.
    for t = 0:T(5)
        T_se = T_se*MatrixExp6(SampleTime(T(5), t)*MatrixLog6(TransInv(T_se)*Tsc_goal*Tce_standoff));
        result = [result; [T_se(1,1:3) ,T_se(2,1:3) ,T_se(3,1:3), T_se(1:3,4)', 1]];
    end
    % step 6 Move the gripper to the final configuration of the object.
    for t = 0:T(6)
        T_se = T_se*MatrixExp6(SampleTime(T(6), t)*MatrixLog6(TransInv(T_se)*Tsc_goal*Tce_grasp));
        result = [result; [T_se(1,1:3), T_se(2,1:3), T_se(3,1:3) ,T_se(1:3,4)', 1]];
    end
    % step 7 Open the gripper.
    for t = 0:T(7)
        result = [result; [T_se(1,1:3), T_se(2,1:3), T_se(3,1:3), T_se(1:3,4)', 0]];
    end
    % step 8 Move the gripper back to the ?standoff? configuration.

    for t = 0:T(8)
        T_se = T_se*MatrixExp6(SampleTime(T(8), t)*MatrixLog6(TransInv(T_se)*Tsc_goal*Tce_standoff));
        result = [result; [T_se(1,1:3), T_se(2,1:3), T_se(3,1:3), T_se(1:3,4)', 0]];
    end
end
