g_vector = [0 0 -9.81];  % [m/s^2]

origin_com_offset = 0.073481;  % [m]
COM_init = [0.13, 0.25, 1.1037];

torso_z_init = COM_init(3) - origin_com_offset;  % [m]

theta_guess = deg2rad([0; 20; -40; 20; 0]);

[theta_init, num_iterations] = inv_kinematics(COM_init(1), COM_init(2), COM_init(3), theta_guess(1), theta_guess(2), theta_guess(3), theta_guess(4), theta_guess(5), 'right');

w = 2*pi*0.5;
t = 0:0.001:9.999;

COM_traj_right = COM_init(2) + 0.1 * sin(w*t);
COM_traj_left = -COM_init(2) + 0.1 * sin(w*t);

joint_traj_right = theta_init .* ones(5, size(t, 2));
joint_traj_left = theta_init .* ones(5, size(t, 2));

num_iterations_right = zeros(1, size(t, 2));
num_iterations_left = zeros(1, size(t, 2));

joint_traj_right(:,1) = theta_init;
joint_traj_left(:,1) = theta_init;

for i = 2:size(COM_traj_right, 2)
    [joint_traj_right(:,i), num_iterations_right(i)] = inv_kinematics(COM_init(1), COM_traj_right(i), COM_init(3), joint_traj_right(1,i-1), joint_traj_right(2,i-1), joint_traj_right(3,i-1), joint_traj_right(4,i-1), joint_traj_right(5,i-1), 'right');
    [joint_traj_left(:,i), num_iterations_left(i)] = inv_kinematics(COM_init(1), COM_traj_left(i), COM_init(3), joint_traj_left(1,i-1), joint_traj_left(2,i-1), joint_traj_left(3,i-1), joint_traj_left(4,i-1), joint_traj_left(5,i-1), 'left');
end

theta_right_timeseries = timeseries(joint_traj_right, t);
theta_left_timeseries = timeseries(joint_traj_left, t);
