% nav_dmp_follower.m
% As following A* path, use DMP and Potential Field to avoid obstacle
function actualPath = nav_dmp_follower(plannedPath, robotConstraints, dynamicObstacles)
    fprintf("DMP-based Path Following with Obstacle Avoidance Started \n");
    
    % Validate Input
    if size(plannedPath, 1) < 3
        error('Planned path must have at least 3 points for DMP learning.');
    end

    % 1. DMP Params Setting
    dt          = 0.01;         % Time step (smaller dt for better stability)
    tau         = 1;            % DMP time scaling factor


    % Use for DMP params
    alpha       = 25;
    beta        = alpha / 4;
    K           = alpha * beta;      % Spring constant for the attractor
    D           = alpha;             % Damping constant for critical damping
    
    % 2. Learn DMP
    % Learn DMP params from plannedPath and receive them in a struct
    dmp_params = learn_dmp_from_path(plannedPath, K, D, tau, dt);
    % dmp_params = learn_dmp_from_path(plannedPath, alpha, beta, tau, dt);
    
    % 3. Initialize Simulation
    startPose           = plannedPath(1, :);
    goalPose            = plannedPath(end, :);

    % State Variables
    y       = startPose(1:3);   % Position
    v       = zeros(1, 3);      % Velocity
    x       = 1;                % Phase Variable, starts at 1
    actualPath = y;             % Store the trajectory
    
    % For debugging store force
    force_history = [];
    

    % Main simulation loop
    % Set a sufficiently large number of iterations. The loop will break
    % based on the termination condition (distance to goal or phase variable).
    max_iter = 10000;
    
    fprintf('Starting DMP execution for max %d iterations \n', max_iter);
    for t = 1:max_iter
        % Calculate forcing term based on the current phase 'x'
        forcing_term = calculate_forcing_term(x, dmp_params);
    
        % Calculate repulsive force from dynamic obstacles
        % obstacle_force = zeros(1,3);

        % Calculate obstacle avoidance term
        obstacle_term = zeros(1, 3);
        if ~isempty(dynamicObstacles)
            % Pass K to limit the maximum force
            % obstacle_force = calculate_obstacle_force(y, dynamicObstacles, K); 
            % obstacle_term = calculate_forcing_term(y, v, dynamicObstacles, alpha, beta);
            obstacle_term = calculate_obstacle_force(y, dynamicObstacles);
        end

        % ===== DMP System Integration =====
        % 1. Transformation system: tau * dv/dt = alpha(beta(g-y) -tau*v) +
        %f(x) + obstacle_term
        g = goalPose(1:3);
        % attractor_term = alpha * (beta * (g - y) - tau * v);
        attractor_term = K * (g - y) - D * v;

        % Calculating acc
        v_dot   = (attractor_term + forcing_term + obstacle_term) / tau;
        
        % v_dot = (K * (goalPose(1:3) - y) - D * v + forcing_term + obstacle_force) / tau;
        
        % 2. Position update: tau * dy/dt = v
        y_dot = v / tau;
        
        % 3. Canonical system: tau * dx/dt = -alpha_x * x
        alpha_x = dmp_params.alpha_x; % Get alpha_x from the learned params
        x_dot = -alpha_x * x / tau;

        % === Euler Integration ===
        v = v + v_dot * dt;
        y = y + y_dot * dt;
        x = x + x_dot * dt; % Update the phase variable
        
        % Ensure x doesn't go negative
        x = max(x, 0);

        % Store Actual Path
        actualPath = [actualPath; y];
        
        % Store force components for analysis
        force_history = [force_history, ...
            norm(attractor_term), norm(forcing_term), norm(obstacle_term)];

        % Check termination Condition
        dist_to_goal = norm(y - goalPose(1:3));
        if dist_to_goal < 0.02
            fprintf('Goal reached at iteration %d. Distance: %.3f, Phase: %.4f \n', t, dist_to_goal, x);
            break;
        end

        % Debug output
        if mod(t, 100) == 0
            fprintf('Iter %d: x = %.3f, pos = [%.2f, %.2f, %.2f], dist_to_goal = %.3f \n', ...
                    t, x, y(1), y(2), y(3), dist_to_goal);
        end
    end

    if t == max_iter
        fprintf('Warning: Maximum iterations reached without converging to the goal.\n');
    end
    fprintf('DMP Path Following Finished with %d points\n', size(actualPath, 1));
end

% ----------------- Local Helper Functions -------------------

% This function learns DMP parameters from a given path.
% It now returns a struct 'dmp_params' containing all necessary parameters.
function dmp_params = learn_dmp_from_path(path, K, D, tau, dt_learn)
    fprintf('Starting DMP learning from Path ... \n');
    
    % Path preprocessing
    path_pos = path(:, 1:3);
    n_points = size(path_pos, 1);
    
    % Calculate velocity and acceleration using central difference
    path_vel = zeros(size(path_pos));
    path_acc = zeros(size(path_pos));
    
    % Forward difference for the first point
    path_vel(1, :) = (path_pos(2, :) - path_pos(1, :)) / dt_learn;
    % Backward difference for the last point
    path_vel(end, :) = (path_pos(end, :) - path_pos(end-1, :)) / dt_learn;
    % Central difference for intermediate points
    for i = 2:n_points-1
        path_vel(i, :) = (path_pos(i+1, :) - path_pos(i-1, :)) / (2 * dt_learn);
    end
    
    % Acceleration calculation (similar to velocity)
    path_acc(1, :) = (path_vel(2, :) - path_vel(1, :)) / dt_learn;
    path_acc(end, :) = (path_vel(end, :) - path_vel(end-1, :)) / dt_learn;
    for i = 2:n_points-1
        path_acc(i, :) = (path_vel(i+1, :) - path_vel(i-1, :)) / (2 * dt_learn);
    end
    
    % Goal Position
    g = path_pos(end, :);
    g_matrix = repmat(g, n_points, 1);
    
    % Calculate target forcing term
    f_target = (tau^2) * path_acc + D * tau * path_vel - K * (g_matrix - path_pos);
    
    % --- Basis functions setup (CRITICAL FIX) ---
    num_basis = 50;
    alpha_x = 25/3;
    
    % Phase variable for the learning duration
    time_steps = linspace(0, 1, n_points);
    x_learn = exp(-alpha_x * time_steps);
    
    % Set up basis function centers and widths correctly
    centers = exp(-alpha_x * linspace(0, 1, num_basis));
    widths = zeros(1, num_basis);
    for i = 1:num_basis - 1
        % Set width based on the distance between centers
        widths(i) = 0.5 / (centers(i+1) - centers(i))^2;
    end
    widths(end) = widths(end - 1); % Set the last width same as the previous one
    
    % Calculate the psi matrix (activation of basis functions over the path)
    psi = zeros(n_points, num_basis);
    for t = 1:n_points
        for i = 1:num_basis
            psi(t, i) = exp(-widths(i) * (x_learn(t) - centers(i))^2);
        end
    end
    
    % --- Learn Weights using Locally Weighted Regression (LWR) ---
    weights = zeros(num_basis, 3);
    for dim = 1:3
        % Target is f_target, scaled by the phase variable x
        target_normalized = f_target(:, dim) ./ (x_learn' + 1e-10);
        
        % LWR calculation
        numerator = sum(psi .* repmat(target_normalized, 1, num_basis), 1);
        denominator = sum(psi, 1);
        denominator(denominator < 1e-10) = 1e-10; % Avoid division by zero
        
        % Calculate and store weights for the current dimension
        weights(:, dim) = (numerator ./ denominator)';
    end
    
    fprintf('DMP learning completed.\n');
    
    % Return all parameters in a single struct
    dmp_params.weights = weights;
    dmp_params.centers = centers;
    dmp_params.widths  = widths;
    dmp_params.alpha_x = alpha_x;
end

% This function calculates the forcing term at a given phase 'x'.
% It now receives all parameters in the 'dmp_params' struct.
function f = calculate_forcing_term(x, dmp_params)
    if x < 1e-6
        f = [0, 0, 0]; 
        return; 
    end
    
    % Get parameters directly from the struct
    weights = dmp_params.weights;
    centers = dmp_params.centers;
    widths  = dmp_params.widths;
    num_basis = length(centers);
    
    % Calculate basis function activations at the current phase 'x'
    psi = zeros(1, num_basis);
    for i = 1:num_basis
        psi(i) = exp(-widths(i) * (x - centers(i))^2);
    end
    
    % Calculate the final forcing term: f = (Sum(psi_i * w_i) / Sum(psi_i)) * x
    psi_sum = sum(psi);
    if psi_sum < 1e-10
        f = [0, 0, 0];
        return;
    end
    
    f = (psi * weights / psi_sum) * x;
end