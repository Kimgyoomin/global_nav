% nav_slicing2D
% for checking collision of torso
function map2D_binary = nav_slicing2D(map3D, torsoHeight, xLimits, yLimits)
    % map3D             : occupancyMap3D object
    % torsoHeight       : [m] Height of Torso Center
    % map2D_binary      : logical 2D map (true = occupied, false = free)
    
    % -> get X/Y by getMapLimits
    % [xLimits, yLimits, ~] = map3D.getMapLimits;

    % xLimits     = map3D.XWorldLimits;   % ex) [-10, 10]
    % yLimits     = map3D.YWorldLimits;   % ex) [-10, 10]
    resolution  = map3D.Resolution;     % cells/m (ex : 10)

    % Create X, Y vector (grid coordinate)
    xVec        = xLimits(1) : 1/resolution : xLimits(2);
    yVec        = yLimits(1) : 1/resolution : yLimits(2);
    [Xg, Yg]    = meshgrid(xVec, yVec);
    N           = numel(Xg);


    % Create Points as z = torsoHeight
    XYZ         = [Xg(:), Yg(:), torsoHeight * ones(N, 1)];

    % getOccupancy ; possiblity [0~1]
    occProb     = map3D.getOccupancy(XYZ);
    occBin      = occProb > map3D.OccupiedThreshold;

    % (nRows x nCols) reshape
    nCols           = numel(xVec);
    nRows           = numel(yVec);
    map2D_binary    = reshape(occBin, [nRows,nCols]);
end