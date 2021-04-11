function [h1, h2, h3, h4] = landingConePlot(x, y, color2, color1, pointsCol)

%mean
[Rm, ad] = mid(x, y);

%max
[RM] = maximum(x, y);

% find min and max angle of landing points
theta = atan2(y, x);
theta_m = min(theta);
theta_M = max(theta);

hold on
h1 = annulus(0, RM, color1, theta_m, theta_M);
h2 = annulus(Rm - ad, Rm + ad, color2, theta_m, theta_M);
alpha(h1, 0.5);
alpha(h2, 0.5);

h3 = plot(0, 0, 'r.', 'MarkerSize', 21);

h4 = plot(x, y, '.', 'Color', pointsCol);


grid on
hold off
axis equal
end

function [RM] = maximum(x, y)
N = length(x);
R = zeros(N, 1);

for i = 1:N
    R(i) = norm([x(i), y(i)]);
    RM = max(R);
end

end

function [Rm, ad] = mid(x, y)
N = length(x);
R = zeros(N, 1);

for i = 1:N
    R(i) = norm([x(i), y(i)]);
    Rm = mean(R);
end

ad = std(R);
end

function h = annulus(r1, r2, col, theta0, theta_end)

Theta = linspace(theta0, theta_end);

x1 = r1*cos(Theta);
y1 = r1*sin(Theta);
x2 = r2*cos(Theta);
y2 = r2*sin(Theta);
h = patch([x1 fliplr(x2)], [y1 fliplr(y2)], col, 'EdgeColor', col);
if nargout == 0
    clear h
end
end