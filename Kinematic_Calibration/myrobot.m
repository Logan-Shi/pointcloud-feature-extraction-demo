classdef myrobot
    %MYROBOT is a robot class that inherit ur10 robot in robotic toolbox
    %(Peter Corke) and integrates matlab built in ROS interface.
    
    properties
       n_dof;
       p_robot;
       dh;
    end
    
    methods
        function obj = myrobot(dh_init)
            %MYROBOT Construct an instance of this class
            %   Detailed explanation goes here
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %                                    PARAMETERS                                 %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %
            % p_robot is a UR10 robot class from Prof. Peter Corke's
            % robotic toolbox
            %
            
            obj.p_robot = p_robot(dh_parameter);      
            % initiate a Peter Corke's robot with dh_init parameter
            obj.dh = dh_init;
        end
        
        function T = fkine(obj, poses)
            % fkine: Forward kinematic of robot
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %                                    PARAMETERS                                 %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %
            % poses: robot joint displacement.
            %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %                                         OUTPUT                                     %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %
            % T is a 4*4 SE(3) Matrix.
            %
            T = obj.p_robot.fkine(poses);
        end
        
        function T = move(obj, target)
            %move: move to a target location represented in SE(3)
            %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %                                         OUTPUT                                     %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %
            % T is a 4*4 SE(3) Matrix that shows the location and
            % orientation of end-effector in robot base frame
            %
            
        end
        
        function p = measure_ball(obj)
            % take a picture and measure the center of the calibration ball
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %                                         OUTPUT                                     %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %
            % P is a 1*3 vector that represent the ball location in camera
            % frame
            %
        end
        
        function As = get_As(obj, pose)
            As = zeros(4,4,obj.n_dof);
            for i=1:obj.n_dof
                As(:,:,i) = obj.p_robot.links(i).A(pose(i));
            end
        end
        
    end
end

