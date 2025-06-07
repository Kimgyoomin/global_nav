% inflate2DManually.m

function map2D_inflated = inflate2DManually(map2D_binary, robotRadius,...
    resolution)
    % map2D_binary      : Logical 2D map(true = occupied, false = true)
    % robotRadius       : [m] radius of torso
    % resolution        : cells/m
    % map2D_inflated    : inflated 2D map(true=inflated obstacle)

    pixelRadius     = round(robotRadius * resolution);
    if pixelRadius < 1
        pixelRadius = 1;
    end

    [nRows, nCols] = size(map2D_binary);
    map2D_inflated = map2D_binary;      % Initialize

    % occupied cell coordinate
    [occRows, occCols]  = find(map2D_binary);
    numOcc              = numel(occRows);

    for idx = 1:numOcc
        i0      = occRows(idx);
        j0      = occCols(idx);
        iMin    = max(i0 - pixelRadius, 1);
        iMax    = min(i0 + pixelRadius, nRows);
        jMin    = max(j0 - pixelRadius, 1);
        jMax    = min(j0 + pixelRadius, nCols);

        for i = iMin : iMax
            for j = jMin : jMax
                if ((i - i0)^2 + (j - j0)^2) <= pixelRadius^2
                    map2D_inflated(i, j)    = true;
                end
            end
        end
    end
end