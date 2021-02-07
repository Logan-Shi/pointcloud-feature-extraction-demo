function [Point_Data, Measure_pose] = get_calibrate_point_location(robot, n_cal, bar_info)
    %GET_CALIBRATE_POINT_LOCATION Move the manipulator to measure points'
    %location on a calibration bar.
    %   The initial pose is manually set, so that the camera is facing the
    %   first hole at the begining.
    %
    %   Idealy, the bar is parallel to the robot's x axis.
    %   If not, an algorithm should be designed to identify the x axis of the
    %   bar.
    %   
    %   The holes' position is measured and recorded in bar_info.
    %   It decides how far the end-effector move each step.
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                                    PARAMETERS                                 %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %bar_info is a struct,
    %bar_info.n_holes describes how many holes is on the bar.
    %bar_info.dis shows how far are holes away from each other.
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                                         OUTPUT                                     %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %Point_Data is an array in shape [3,bars_info.n_holes,n_cal].
    %   Each column (which is 3) shows the measured location of that hole
    %   from the scanner's view.
    %Measure_pose records the robot.n_dof dimensional joint displacement
    %   while measuring each hole on the bar.
    
    
    Point_Data = zeros(3,bar_info.n_holes,n_cal);
    Measure_pose = zeros(robot.n_dof, bar_info.n_holes, n_cal);
    
end

