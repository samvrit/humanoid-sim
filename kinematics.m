syms theta1 theta2 theta3 theta4 theta5
syms a alpha d theta

                %  |a      |alpha      |d      |theta
dh_right =       [  0.13    0           0.075   0;
                    0       -pi/2       0       pi/2;
                    0       -pi/2       0       -pi/2 + theta1;
                    0.45    0           0       theta2;
                    0.45    0           0       theta3;
                    0       -pi/2       0.1     theta4;
                    0.273   0           0       theta5;
                    0.15    -pi/2       0       -pi/2;
                    0       0           0       -pi/2];

                %  |a      |alpha      |d      |theta
dh_left =        [  0.13    0           0.075   0;
                    0       -pi/2       0       pi/2;
                    0       -pi/2       0       -pi/2 + theta1;
                    0.45    0           0       theta2;
                    0.45    0           0       theta3;
                    0       -pi/2       -0.1    theta4;
                    0.273   0           0       theta5;
                    -0.15   -pi/2       0       -pi/2;
                    0       0           0       -pi/2];

A(a, alpha, d, theta) = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha)  a*cos(theta);
                         sin(theta) cos(theta)*cos(alpha)  -cos(theta)*sin(alpha) a*sin(theta);
                               0          sin(alpha)              cos(alpha)              d;
                               0              0                       0                   1];

% Transformation matrix for each individual link (Right Leg)
A1_right = A(dh_right(1, 1), dh_right(1, 2), dh_right(1, 3), dh_right(1, 4)) * A(dh_right(2, 1), dh_right(2, 2), dh_right(2, 3), dh_right(2, 4)) * A(dh_right(3, 1), dh_right(3, 2), dh_right(3, 3), dh_right(3, 4));
A2_right = A(dh_right(4, 1), dh_right(4, 2), dh_right(4, 3), dh_right(4, 4));
A3_right = A(dh_right(5, 1), dh_right(5, 2), dh_right(5, 3), dh_right(5, 4));
A4_right = A(dh_right(6, 1), dh_right(6, 2), dh_right(6, 3), dh_right(6, 4));
A5_right = A(dh_right(7, 1), dh_right(7, 2), dh_right(7, 3), dh_right(7, 4)) * A(dh_right(8, 1), dh_right(8, 2), dh_right(8, 3), dh_right(8, 4)) * A(dh_right(9, 1), dh_right(9, 2), dh_right(9, 3), dh_right(9, 4));

% Transformation matrix for each individual link (Left Leg)
A1_left = A(dh_left(1, 1), dh_left(1, 2), dh_left(1, 3), dh_left(1, 4)) * A(dh_left(2, 1), dh_left(2, 2), dh_left(2, 3), dh_left(2, 4)) * A(dh_left(3, 1), dh_left(3, 2), dh_left(3, 3), dh_left(3, 4));
A2_left = A(dh_left(4, 1), dh_left(4, 2), dh_left(4, 3), dh_left(4, 4));
A3_left = A(dh_left(5, 1), dh_left(5, 2), dh_left(5, 3), dh_left(5, 4));
A4_left = A(dh_left(6, 1), dh_left(6, 2), dh_left(6, 3), dh_left(6, 4));
A5_left = A(dh_left(7, 1), dh_left(7, 2), dh_left(7, 3), dh_left(7, 4)) * A(dh_left(8, 1), dh_left(8, 2), dh_left(8, 3), dh_left(8, 4)) * A(dh_left(9, 1), dh_left(9, 2), dh_left(9, 3), dh_left(9, 4));

% Kinematic chain (Right Leg)
T1_right = A1_right;
T2_right = T1_right * A2_right;
T3_right = T2_right * A3_right;
T4_right = T3_right * A4_right;
T5_right = T4_right * A5_right;

% Kinematic chain (Left Leg)
T1_left = A1_left;
T2_left = T1_left * A2_left;
T3_left = T2_left * A3_left;
T4_left = T3_left * A4_left;
T5_left = T4_left * A5_left;

%% Jacobian
% z-axis (Right Leg)
z0 = [0; 0; 1];
z1_right = T1_right(1:3,3);
z2_right = T2_right(1:3,3);
z3_right = T3_right(1:3,3);
z4_right = T4_right(1:3,3);
z5_right = T5_right(1:3,3);

% z-axis (Left Leg)
z1_left = T1_left(1:3,3);
z2_left = T2_left(1:3,3);
z3_left = T3_left(1:3,3);
z4_left = T4_left(1:3,3);
z5_left = T5_left(1:3,3);

% Origin positions (Right Leg)
o0 = [0; 0; 0];
o1_right = T1_right(1:3,4);
o2_right = T2_right(1:3,4);
o3_right = T3_right(1:3,4);
o4_right = T4_right(1:3,4);
o5_right = T5_right(1:3,4);

% Origin position (Left Leg)
o1_left = T1_left(1:3,4);
o2_left = T2_left(1:3,4);
o3_left = T3_left(1:3,4);
o4_left = T4_left(1:3,4);
o5_left = T5_left(1:3,4);

% Jacobian Calculation
Jv_right = [cross(z0, (o5_right - o0)), cross(z1_right, (o5_right - o1_right)), cross(z2_right, (o5_right - o2_right)), cross(z3_right, (o5_right - o3_right)), cross(z4_right, (o5_right - o4_right))];
Jw_right = [z0 z1_right z2_right z3_right z4_right];
J_right = [Jv_right; Jw_right];

Jv_left = [cross(z0, (o5_left - o0)), cross(z1_left, (o5_left - o1_left)), cross(z2_left, (o5_left - o2_left)), cross(z3_left, (o5_left - o3_left)), cross(z4_left, (o5_left - o4_left))];
Jw_left = [z0 z1_left z2_left z3_left z4_left];
J_left = [Jv_left; Jw_left];