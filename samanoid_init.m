g_vector = [0 0 -9.81*0];  % [m/s^2]

torso_z_init = 1.25;  % [m]

theta1_init = 0;   % [deg]
theta2_init = 20;  % [deg]
theta3_init = -40; % [deg]
theta4_init = 20;  % [deg]
theta5_init = 0;   % [deg]

right_theta1_init = 0;   % [deg]
right_theta2_init = 20;  % [deg]
right_theta3_init = -40; % [deg]
right_theta4_init = 20;  % [deg]
right_theta5_init = 0;   % [deg]

left_theta1_init = 0;   % [deg]
left_theta2_init = 25;  % [deg]
left_theta3_init = -50; % [deg]
left_theta4_init = 25;  % [deg]
left_theta5_init = 0;   % [deg]

torso_desired_z = 1.2;
torso_desired_x = 0.13;
torso_desired_y = 0.0;

theta_new = inv_kinematics(torso_desired_x, torso_desired_y, torso_desired_z, deg2rad(theta1_init), deg2rad(theta2_init), deg2rad(theta3_init), deg2rad(theta4_init), deg2rad(theta5_init));

theta_new_deg = rad2deg(theta_new);
