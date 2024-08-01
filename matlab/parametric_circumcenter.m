%% Forward Kinematics
function [pos] = parametric_circumcenter(u1,u2,u3,h)
    % Normal unit vector of the plane formed by the centres S1,S2,S3
    n = cross(u3-u1,u2-u1);     %% Leg down % reverse if upside down
    e = n./norm(n);
    
    % Parametric circumcenter
    % Compute vectors AB and AC
    AB = u2 - u1;
    AC = u3 - u1;
    % Compute the normal vector to the plane of the triangle (N = AB x AC)
    N = cross(AB, AC);
    % Compute midpoints of AB and AC
    D = (u1 + u2) / 2;
    E = (u1 + u3) / 2;
    % Compute vectors from the midpoints in the direction perpendicular to AB and AC
    v1 = cross(N, AB);
    v2 = -cross(N, AC);
    % % Set up the matrix equation (v1, -v2) * [t; s] = E - D
    vector = E - D;
    % Use parametric form of line equations
    t = (v2(2)*vector(1) - v2(1)*vector(2))/(v1(1)*v2(2) - v2(1)*v1(2));

    % Calculate the circumcenter using the parametric equation of one of the lines
    c = D + t * v1;

    % Distance between centre C and position p
    rho = sqrt(h^2 - abs((u1 - c)'*(u1 - c)));

    % Position
    pos = c + rho*e;
end