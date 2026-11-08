% test_setup_diagram.m - top view of the test setup:
% UMA-16 array at the centre, 7 speaker positions on a 1 m semicircle (-90 to +90 deg)

angles = -90:30:90;      % speaker positions (deg), 0 = straight ahead
R      = 1;              % distance from array centre (m)
d      = 0.042;          % mic spacing (m)

figure('Color', 'w', 'Position', [100 100 900 560]);
hold on; axis equal; axis off;

% dashed 1 m arc
th = linspace(-90, 90, 200);
plot(R*sind(th), R*cosd(th), '--', 'Color', [0.75 0.75 0.75], 'LineWidth', 1);

% 0 deg reference (broadside) line
plot([0 0], [0 1.25], ':', 'Color', [0.6 0.6 0.6]);

for a = angles
    x = R*sind(a);  y = R*cosd(a);

    % line from the array to the speaker
    plot([0 x], [0 y], '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2);

    % speaker: a rounded box with a cone, rotated to face the array
    w = 0.09;  h = 0.13;
    bx = [-w w w -w]/2;  by = [-h -h h h]/2;
    rot = [cosd(-a) -sind(-a); sind(-a) cosd(-a)];
    P = rot * [bx; by];
    patch(P(1,:)+x, P(2,:)+y, [0.35 0.35 0.35], 'EdgeColor', 'none');
    c = rot * [0; -0.02];                                % woofer centre
    t = linspace(0, 2*pi, 40);
    patch(x+c(1)+0.03*cos(t), y+c(2)+0.03*sin(t), 'w', 'EdgeColor', 'none');
    patch(x+c(1)+0.015*cos(t), y+c(2)+0.015*sin(t), [0.35 0.35 0.35], 'EdgeColor', 'none');
    c2 = rot * [0; 0.04];                                % tweeter
    patch(x+c2(1)+0.012*cos(t), y+c2(2)+0.012*sin(t), 'w', 'EdgeColor', 'none');

    % angle label just outside the speaker
    lx = 1.17*R*sind(a);  ly = 1.17*R*cosd(a);
    if a > 0, lbl = sprintf('+%d°', a); else, lbl = sprintf('%d°', a); end
    text(lx, ly, lbl, 'HorizontalAlignment', 'center', 'FontSize', 13, 'FontWeight', 'bold');
end

% the array: a board with 4 mics (shown in a row, 42 mm apart)
rectangle('Position', [-0.09 -0.03 0.18 0.035], 'FaceColor', [0.85 0.85 0.85], 'EdgeColor', 'k');
mx = (-1.5:1:1.5)*d;
plot(mx, zeros(size(mx)), 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.2 0.4 0.8], 'MarkerEdgeColor', 'k');
text(0, -0.08, 'UMA-16 (4 mics, 42 mm spacing)', 'HorizontalAlignment', 'center', 'FontSize', 11);

% distance label on the 0 deg line
text(0.03, 0.55, '1 m', 'FontSize', 12, 'FontAngle', 'italic');

xlim([-1.3 1.3]); ylim([-0.15 1.3]);
title('Test setup (top view): speaker 1 m from the array centre', 'FontSize', 14);
hold off;

print(gcf, 'test_setup_diagram.png', '-dpng', '-r300');