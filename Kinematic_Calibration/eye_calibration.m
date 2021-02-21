function H = eye_calibration(robot, n_samples)
    %EYE_CALIBRATION Hand-eye parameter Calibration
    % The return matrix H is an SE(3)
    % param: robot is a robot class that could execute motion and solve fkine &
    % ikine
    % param: n_samples descirbes how many samples we take to solve the
    % optimization problem. n_samples should be larger than 3 the make sure
    % there are enought equations to solve all the unknowns (6 unknowns).


    init_robot(robot);  %Manually reset the robot so that the eye is looking at the target ball

    % In short, we need to make sure that the generated poses should guarentee
    % that the target is always in sight
    poses = gen_eye_calibration_pos(robot, n_samples);
    % I think we should not use Inverse kinematic here, since the DH parameters
    % are not calibrated yet.

    % H is in form:[n,o,a,p;0,0,0,0]
    % where n, o, a, p are all 3*1 vector,
    % with constraints ||a||=1, ||n|| = 1, ||o||=1
    %                               a'*n=0, n'*o=0, o'*a=0;

    % Optimize Variable
    n  = optimvar('n',3,1);
    o  = optimvar('o',3,1);
    a  = optimvar('a',3,1);
    p_H = optimvar('p',3,1);
    x0 = struct('n',n0,'o',o0,'a',a0,'p',p0);
    % constraints
    constraint(1) = n.*n ==1;
    constraint(2) = a.*a ==1;
    constraint(3) = o.*o ==1;
    constraint(4) = a.*n ==0;
    constraint(5) = n.*o ==0;
    constraint(6) = o.*a ==1;

    R_H = [n o a];
    P_R = optimexpr(3,n_sample);
    for i=1:n_samples
        current_pose = poses(i,:);
        T_N = robot.move(current_pose);
        P = robot.measure_ball();
        R_N = T_N(1:3,1:3);
        P_R(:,i)=R_N*R_H*P+R_N*p_H;
    end
    
    E = optimexpr(1);
    for i=1:n_samples
        for j=(i+1):n_samples
            E = E + (P_R(:,i)-P_R(:,j)).'*(P_R(:,i)-P_R(:,j));
        end
    end
    
    prob = optimproblem('Objective', E,'Constraints',constraint);
    tic;
    sol = solve(prob,x0);
    toc;
    H = [sol.n, sol.o, sol.a, sol.p;0,0,0,1];
end

