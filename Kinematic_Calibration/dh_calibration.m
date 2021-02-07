function dh_calibration(robot, bar_info, tol, n_cal)
    %DH_CALIBRATION Main function for dh parameter calibration
    %   Iteritively solving linear programming problem until the requirement is
    %   met.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                                    PARAMETERS                                 %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %robot is a robot class that could execute motion and solve fkine &
    % ikine
    % tol is the tolerence for error.
    % n_cal is the number of time to measure in one iteration.
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                                       PROCESS                                     %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    % We have a pre-manufactured bar with several holes on it.
    % The 2D/3D distance between each two holes are measured
    % by a CMM, which may reach an accuracy of over 0.02mm.
    % These holes are numbered as hole_1...hole_n, with distance
    % between each two hole measured as x_12, x_13.
    % 
    % In this piece of code, we assume the bar has 6 holes, results in
    % 15 different combinations.
    % 
    % Then, using the scanner mounted on the end of the robot, we
    % measure the location of each hole n_cal times, which leads to 
    % 15*n_cal data points for calibration.
    %
    % With the data gathered above, we can solve a linear programming
    % problem to calibrate the dh parameters.
    
    delta_x =ones(n_cal, 1);    % make sure that the initial mean(delta_x) > tol to start the loop
    
    As_All = zeros(4,4,robot.n_dof, bars_info.n_holes, n_cal);
    while mean(delta_x) >= tol
        for i = 1:n_cal
        [point_location_measure, measure_pose] = get_calibrate_point_location(robot, n_cal, bar_info);
        As = robot.
    end
    
    
    
    DH = [];
end

