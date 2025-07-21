% This function calculates the repulsive force from an obstacle.
% It now receives K to limit the maximum force.
function force = calculate_obstacle_force(robot_pos, obstacle)
    % Params for Sperichal Potential field
    d0   = 2;      % Distance of influence [m]. Force starts acting inside this distance
    eta = 1;        % Repulsive force gain. U can Tune this value
    
    % Relative position between robot and obstacle center [Vector]
    rel_pos = robot_pos - obstacle.pos;
    % Distance from obstacle to robot
    dist = norm(rel_pos);
              

    % If the robot is outside the value v Err distance ignore
    if dist > d0 || dist < 1e-6 % Change dist value for safety
        force = [0,0,0];
        return;
    end
    
    % Direction of repulsive force (keep away from obstacle)
    grad_d = rel_pos / dist;

    % Calculate the magnitude of the repulsive force
    % The force increases quadratically as the robot gets closer
    force_magnitude = eta * (1/dist - 1/d0) * (1/dist^2);

    % Final force vector
    force = force_magnitude * grad_d;
end