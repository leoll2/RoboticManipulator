%% Plot a 3D cube
% c: center coordinates
% l: length of the edges
function [] = plotcube(c, l)
    cx = c(1);
    cy = c(2);
    cz = c(3);
    lx = l(1);
    ly = l(2);
    lz = l(3);

    a = -pi : pi/2 : pi;                                % Define Corners
    ph = pi/4;                                          % Define Angular Orientation (‘Phase’)
    x = [cx; cx] + [lx/2*cos(a+ph); lx/2*cos(a+ph)]/cos(ph);
    y = [cy; cy] + [ly/2*sin(a+ph); ly/2*sin(a+ph)]/sin(ph);
    z = [cz; cz] + [lz/2*-ones(size(a)); lz/2*ones(size(a))];
    surf(x, y, z, 'FaceColor', 'g')                      % Plot Cube
    patch(x', y', z', 'r')                               % Make Cube Appear Solid
    hold off
end