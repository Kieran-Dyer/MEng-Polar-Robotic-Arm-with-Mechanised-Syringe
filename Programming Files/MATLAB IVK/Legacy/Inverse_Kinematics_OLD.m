% Clear workspace and figure
clc; clear; close all;

%% ===== VARIABLES =====

% Constants
radToDeg = 180 / pi;
degToRad = pi / 180;

% Target Position (Modify these values to test)
X_pos = 10;  % in mm
Y_pos = 5;  % in mm
Z_pos = 7;  % in mm

% Offsets (in mm)
% Orange Vector
O_X_offset = -117.316;
O_Y_offset = -88.741;
O_Z_offset = 45.064;

% Purple Vector
P_L_4 = 24.884;
P_Y_offset = 88.741;
P_Z_3 = -106.808;

% Light Blue Vector
B_L_1 = 35;

% Arm Length
A = 175;

% Predefined angle offsets (converted to radians)
theta_offset_1 = degToRad * 123.13;
theta_offset_2 = degToRad * 27.88;
theta_offset_3 = degToRad * -10;
theta_offset_4 = degToRad * -28.99;


%% ===== INVERSE KINEMATICS ====

% ===== Waist Angle (XY Plane) =====
theta_3 = atan2((Y_pos - O_Y_offset), (X_pos - O_X_offset));
BigRadius_RT = hypot(X_pos - O_X_offset, Y_pos - O_Y_offset);  % Stable computation
theta_offset_5 = asin(P_Y_offset / BigRadius_RT);
targetAngleWaist = theta_3 - theta_offset_5;

% ===== Shoulder and Elbow Angles (RZ Plane) =====
Radius = sqrt(BigRadius_RT^2 - P_Y_offset^2);

% Elbow angle q_2 relative to q_1 (geometry of triangle)
q_2 = acos(((Radius - B_L_1 - P_L_4)^2 + (Z_pos - O_Z_offset - P_Z_3)^2) / (2 * A * A) - 1);

% Shoulder angle q_1 measured from horizontal, clockwise negative
q_1 = atan2((Z_pos - O_Z_offset - P_Z_3), (Radius - B_L_1 - P_L_4)) - atan2(A * sin(q_2), A + A * cos(q_2));

% Convert to horizontal-referenced angles
theta_1 = q_2 + q_1; % 132.119 + (-18.9878 = 113.13deg, changed q_1 to atan2(y/x) which references horizontal, rather than atan2(x/y) which references vertical
theta_2 = q_1; % -18.9878deg, changed q_1 to atan2(y/x) which references horizontal, rather than atan2(x/y) which references vertical

% Constrain upper arm to not exceed physical limitations
theta_1 = min(max(theta_1, 0), theta_offset_1);

% Calculate the min and max angles for theta_2 based on theta_1
theta_2_min = -pi + theta_1 + theta_offset_2;
theta_2_max = theta_1 + theta_offset_3;

% Constrain theta_2 to not exceed physical limitations
theta_2 = min(max(theta_2, theta_2_min), theta_2_max);

% Final target angles for motors (relative to offsets)
targetAngleShoulder = theta_1 - theta_offset_1;
targetAngleElbow    = -theta_offset_4 + theta_2;

% Convert from radians to degrees for display
targetAngleWaist_deg    = targetAngleWaist * radToDeg;
targetAngleShoulder_deg = targetAngleShoulder * radToDeg;
targetAngleElbow_deg    = targetAngleElbow * radToDeg;

%% ==== XY PLANE PLOTTING ====

% Compute key points
zero_point = [0, 0];  % Fixed origin

waist_joint = [O_X_offset, O_Y_offset];  % Waist position

% Shoulder joint position from waist
shoulder_joint = waist_joint + [B_L_1 * cos(targetAngleWaist), B_L_1 * sin(targetAngleWaist)];

% Elbow joint position from shoulder (rotate by -theta_1 from waist angle)
elbow_joint = shoulder_joint + [A * cos(theta_1) * cos(targetAngleWaist), A * cos(theta_1) * sin(targetAngleWaist)];

% Wrist joint position from elbow (rotate by -theta_2 from waist angle)
wrist_joint = elbow_joint + [A * cos(theta_2) * cos(targetAngleWaist), A * cos(theta_2) * sin(targetAngleWaist)];

% End effector calculation (P_L_4 vector)
P_L_4_x = P_L_4 * cos(targetAngleWaist);
P_L_4_y = P_L_4 * sin(targetAngleWaist);
P_Y_x   = -P_Y_offset * sin(targetAngleWaist);
P_Y_y   =  P_Y_offset * cos(targetAngleWaist);

end_effector = wrist_joint + [P_L_4_x + P_Y_x, P_L_4_y + P_Y_y];

% Dark blue vector (direct from Zero Point to End Effector)
direct_vector = end_effector;

% Plotting
figure;
hold on;
grid on;
axis equal;
xlabel('X (mm)');
ylabel('Y (mm)');
title('Top-Down View (XY Plane)');

% Plot all segments with different colours
plot([zero_point(1), waist_joint(1)],     [zero_point(2), waist_joint(2)],     'color', [1, 0.5, 0], 'LineWidth', 2); % Orange Vector
plot([waist_joint(1), shoulder_joint(1)], [waist_joint(2), shoulder_joint(2)], 'c', 'LineWidth', 2);                  % Light Blue Vector
plot([shoulder_joint(1), elbow_joint(1)], [shoulder_joint(2), elbow_joint(2)], 'r', 'LineWidth', 2);                  % Red Vector
plot([elbow_joint(1), wrist_joint(1)],    [elbow_joint(2), wrist_joint(2)],    'g', 'LineWidth', 2);                  % Green Vector
plot([wrist_joint(1), end_effector(1)],   [wrist_joint(2), end_effector(2)],   'm', 'LineWidth', 2);                  % Purple Vector
plot([zero_point(1), end_effector(1)],    [zero_point(2), end_effector(2)],    'b--', 'LineWidth', 1.5);              % Dark Blue Vector

% Scatter plot for joint positions
scatter([zero_point(1), waist_joint(1), shoulder_joint(1), elbow_joint(1), wrist_joint(1), end_effector(1)], ...
        [zero_point(2), waist_joint(2), shoulder_joint(2), elbow_joint(2), wrist_joint(2), end_effector(2)], ...
        50, 'k', 'filled');

% Labels
text(waist_joint(1), waist_joint(2),       ' Waist',        'FontSize', 10);
text(shoulder_joint(1), shoulder_joint(2), ' Shoulder',     'FontSize', 10);
text(elbow_joint(1), elbow_joint(2),       ' Elbow',        'FontSize', 10);
text(wrist_joint(1), wrist_joint(2),       ' Wrist',        'FontSize', 10);
text(end_effector(1), end_effector(2),     ' End Effector', 'FontSize', 10);

hold off;

%% ==== RZ PLANE PLOTTING ====

% Compute key points
zero_point = [0, 0];  % Fixed origin

% Waist joint in RZ coordinates (Orange vector)
waist_R = O_X_offset * cos(targetAngleWaist) + O_Y_offset * sin(targetAngleWaist);
waist_joint_RZ = [waist_R, O_Z_offset];

% Shoulder joint (Light blue vector): constant R = B_L_1 forward from waist
shoulder_joint_RZ = waist_joint_RZ + [B_L_1, 0];

% Elbow joint (Red vector): length A, at angle theta_1 (inverted to rotate anticlockwise from horizontal)
elbow_joint_RZ = shoulder_joint_RZ + A * [cos(theta_1), sin(theta_1)];

% Wrist joint (Green vector): length A, at angle theta_2 (inverted)
wrist_joint_RZ = elbow_joint_RZ + A * [cos(theta_2), sin(theta_2)];

% End effector (Purple vector): add P_L_4 in R and P_Z_3 in Z
end_effector_RZ = wrist_joint_RZ + [P_L_4, P_Z_3];

% Plotting RZ plane
figure;
hold on;
grid on;
axis equal;
xlabel('R (mm)');
ylabel('Z (mm)');
title('Side View (RZ Plane)');

% Plot each segment
plot([zero_point(1), waist_joint_RZ(1)], [zero_point(2), waist_joint_RZ(2)], 'color', [1, 0.5, 0], 'LineWidth', 2); % Orange
plot([waist_joint_RZ(1), shoulder_joint_RZ(1)], [waist_joint_RZ(2), shoulder_joint_RZ(2)], 'c', 'LineWidth', 2);    % Light Blue
plot([shoulder_joint_RZ(1), elbow_joint_RZ(1)], [shoulder_joint_RZ(2), elbow_joint_RZ(2)], 'r', 'LineWidth', 2);    % Red
plot([elbow_joint_RZ(1), wrist_joint_RZ(1)], [elbow_joint_RZ(2), wrist_joint_RZ(2)], 'g', 'LineWidth', 2);          % Green
plot([wrist_joint_RZ(1), end_effector_RZ(1)], [wrist_joint_RZ(2), end_effector_RZ(2)], 'm', 'LineWidth', 2);        % Purple
plot([zero_point(1), end_effector_RZ(1)], [zero_point(2), end_effector_RZ(2)], 'b--', 'LineWidth', 1.5);            % Blue Dashed

% Scatter plot for joint positions
scatter([zero_point(1), waist_joint_RZ(1), shoulder_joint_RZ(1), elbow_joint_RZ(1), wrist_joint_RZ(1), end_effector_RZ(1)], ...
        [zero_point(2), waist_joint_RZ(2), shoulder_joint_RZ(2), elbow_joint_RZ(2), wrist_joint_RZ(2), end_effector_RZ(2)], ...
        50, 'k', 'filled');

% Labels
text(waist_joint_RZ(1), waist_joint_RZ(2), ' Waist', 'FontSize', 10);
text(shoulder_joint_RZ(1), shoulder_joint_RZ(2), ' Shoulder', 'FontSize', 10);
text(elbow_joint_RZ(1), elbow_joint_RZ(2), ' Elbow', 'FontSize', 10);
text(wrist_joint_RZ(1), wrist_joint_RZ(2), ' Wrist', 'FontSize', 10);
text(end_effector_RZ(1), end_effector_RZ(2), ' End Effector', 'FontSize', 10);

hold off;