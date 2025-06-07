% nav_robot_modeling.m
% Function which defines robot modeling

function[robot, robotConstraints] = nav_robot_modeling()
    % nav_robot_modeling : Parsing Unitree Go2 robot model from URDF file
    % Defines robot's constraints which needs for Hybrid A* Algorithm
    % output: robot - loaded robotics.RigidBodyTree
    % robot
    fprintf('--- Robot Modeling Initialize --- \n');

    % 1. Get Robot URDF File
    % Set URDF File Directory from where main.m exist
    % URDF File will be exist under
    % robots/go2_description/urdf/go2_description.urdf
    urdfFilePath = 'robots/go2_description/urdf/go2_description.urdf';

    try
        fprintf('Loading URDF : %s\n', urdfFilePath);
        robot = importrobot(urdfFilePath);
        fprintf('Robot Model Loaded \n');

        % Robot model visualization
        figure('Name', 'Unitree Go2 Robot Model');
        show(robot, 'Frames', 'off');       % Too much resources -> turn off
        title('Unitree Go2 Robot model from URDF');
        view(3);
        light;


    catch ME
        frpintf('Error occured while loading URDF file: %s\n', ME.message);

        % if error occured, just make simple rigit body
        robot = robotics.RigidBodyTree;
        base = robotics.RigitBody('base');

        % Unitree Go2 sample creation : Length(0.58m), Width(0.29m),
        % Height(0.22m)
        addCollision(base, robotics.CollisionBox(0.6, 0.3, 0.4));
        addBody(robot, base, 'base');

        % Sample model visualization
        figure('Name', 'Simple Robot Model (Fallback)');
        show(robot);
        title('Simple Robot Model (Fallback)');
        view(3);
    end

    % 2. Set Robot Constraints for Hybrid A*
    % This value should be changed based on 'REAL PROPERTIES'
    % IN URDF, collision box size[0.3762 0.0935 0.114]
    robotConstraints.bodyRadius         = 0.35;         % [m]
    % Max Linear Velocity (5m/s in lab, 3.5m/s in broshure
    robotConstraints.maxLinearVelocity  = 3.5;          % [m/s]
    % It should be changed after.....checking yaw vel
    robotConstraints.maxAngularVelocity = pi/6;         % [rad/s]
    % Ability for robot which robot can turn in min radius / 회전선회최소반경
    robotConstraints.minTurningRadius   = 0.5;          % [m]
    % Heights which robot can step / Change based on Control methods
    robotConstraints.maxStepHeight     = 0.2;          % [m]
    % max Slope Degree robot can go / According to broshure
    robotConstraints.maxSlope           = deg2rad(40);   % [rad]

end
    
