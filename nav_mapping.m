% nav_mapping.m file
% script which define function related to mapping

% function map3D = nav_mapping()
function [map3D, xLimits, yLimits, zLimits] = nav_mapping()
% nav_mapping : Create 3D Occupancy Grid Map and Initialize
% Obstacle information will be expanded in this function
% Output : Initialized occupancy3D 
    
    % 1. Mapping resolution setting
    % Cell texture will be 10cm for each
    resolution  = 10;    % cells / meter
    map3D       = occupancyMap3D(resolution);

    % 2. Set Map's geographical setting
    % 20m * 20m * 5m (Width Length Height)
    minX = -10; maxX = 10;
    minY = -10; maxY = 10;
    minZ = 0;   maxZ = 5;

    % all of grid cell will be 'free space'
    % Based on Map resolution, sampling points by 0.1m
    [X_grid, Y_grid, Z_grid] = meshgrid(minX:1/resolution:maxX, ...
                                        minY:1/resolution:maxY, ...
                                        minZ:1/resolution:maxZ);
    all_points_in_range = [X_grid(:), Y_grid(:), Z_grid(:)];

    % Set sampling points as free space -> map occupancy and size
    % fprintf('Initializing Map (Initializing free space)');
    map3D.setOccupancy(all_points_in_range, 0);     % All points will be free space (0)
    % fprintf('Finished');
    

    % Example 1 : Create Wall in the center of the Map (Robot must go outer)
    % X : 3 ~ 10, Y : 2 ~ 2.5, Z : 0 ~ 3
    obstacle1_x_range = 3:1/resolution:10;
    obstacle1_y_range = 2:1/resolution:2.5;
    obstacle1_z_range = 0:1/resolution:3;
    [X1, Y1, Z1] = meshgrid(obstacle1_x_range, obstacle1_y_range, ...
        obstacle1_z_range);
    obstacle1_points = [X1(:), Y1(:), Z1(:)];
    map3D.setOccupancy(obstacle1_points, 1);    % set as occupied space

    % Example 2 : Create Wall in the center of the Map
    % X : 5.5 ~ 6, Y : 4, 9, Z : 0 ~ 3
    obstacle2_x_range = 5.5:1/resolution:6;
    obstacle2_y_range = 4:1/resolution:9;
    obstacle2_z_range = 0:1/resolution:3;
    [X2, Y2, Z2] = meshgrid(obstacle2_x_range, obstacle2_y_range, ...
        obstacle2_z_range);
    obstacle2_points = [X2(:), Y2(:), Z2(:)];
    map3D.setOccupancy(obstacle2_points, 1);    % set as occupied space

    % Example 3 : Create Wall in the center of the Map
    % X : 

    fprintf('Obstacle Set Ready!!!!!\n\n');
    
    xLimits     = [minX, maxX];
    yLimits     = [minY, maxY];
    zLimits     = [minZ, maxZ];
    % 초기화된 맵의 실제 범위 확인 (참고용)
    % mapLimits = map3D.getMapLimits();
    % fprintf('생성된 맵의 실제 X Limits: [%.2f, %.2f] m\n', minX, maxX);
    % fprintf('생성된 맵의 실제 Y Limits: [%.2f, %.2f] m\n', minY, maxY);
    % fprintf('생성된 맵의 실제 Z Limits: [%.2f, %.2f] m\n', minZ, maxZ);

    % % 초기 맵 시각화 (빈 공간 확인)
    % figure('Name', 'Initial Empty 3D Occupancy Map');
    % show(map3D);
    % title('Initial Empty 3D Occupancy Grid Map (20x20x5m)');
    % xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    % view(3); % 3D 뷰

end
% 250604 4