function [X, Y, Z] = RectangularPrism2(center, side1, side2, side3, numPoints)
    % center: [x, y, z] coordinates of the center of the prism
    % side1: length along x-axis
    % side2: length along y-axis
    % side3: length along z-axis
    % numPoints: number of points along each edge of the prism (optional)

    if nargin < 5
        numPoints = 10; % Default number of points
    end

    % Calculate half lengths
    halfSide1 = side1 / 2;
    halfSide2 = side2 / 2;
    halfSide3 = side3 / 2;

    % Create a grid of points on each face of the prism
    x = linspace(-halfSide1, halfSide1, numPoints);
    y = linspace(-halfSide2, halfSide2, numPoints);
    z = linspace(-halfSide3, halfSide3, numPoints);
    
    % Generate matrices for each face of the rectangular prism
    % Bottom face
    [X1, Y1] = meshgrid(x, y);
    Z1 = -halfSide3 * ones(size(X1)); % Z is constant for the bottom face

    % Top face
    [X2, Y2] = meshgrid(x, y);
    Z2 = halfSide3 * ones(size(X2)); % Z is constant for the top face

    % Front face (Z constant)
    [X3, Z3] = meshgrid(x, z);
    Y3 = halfSide2 * ones(size(X3)); % Y is constant for the front face

    % Back face (Z constant)
    [X4, Z4] = meshgrid(x, z);
    Y4 = -halfSide2 * ones(size(X4)); % Y is constant for the back face

    % Left face (Y constant)
    [Y5, Z5] = meshgrid(y, z);
    X5 = -halfSide1 * ones(size(Y5)); % X is constant for the left face

    % Right face (Y constant)
    [Y6, Z6] = meshgrid(y, z);
    X6 = halfSide1 * ones(size(Y6)); % X is constant for the right face

    % Combine all the points and shift them to the center
    X = [X1; X2; X3; X4; X5; X6] + center(1);
    Y = [Y1; Y2; Y3; Y4; Y5; Y6] + center(2);
    Z = [Z1; Z2; Z3; Z4; Z5; Z6] + center(3);
end
