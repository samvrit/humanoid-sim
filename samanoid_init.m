g_vector = [0 0 -9.81];  % [m/s^2]

origin_com_offset = 0.073481;  % [m]
COM_init = [0.13, 0.25, 1.1037];

torso_z_init = COM_init(3) - origin_com_offset;  % [m]

theta_guess = deg2rad([0; 20; -40; 20; 0]);

w = 2*pi*0.5;
t = 0:0.001:9.999;

COM_traj_right(1,:) = COM_init(1) .* ones(1, size(t, 2));
COM_traj_right(2,:) = COM_init(2) + 0.1 * sin(w*t);
COM_traj_right(3,:) = COM_init(3) .* ones(1, size(t, 2));

COM_traj_left(1,:) = COM_init(1) .* ones(1, size(t, 2));
COM_traj_left(2,:) = -COM_init(2) + 0.1 * sin(w*t);
COM_traj_left(3,:) = COM_init(3) .* ones(1, size(t, 2));

COM_traj_left_timeseries = timeseries(COM_traj_left, t);
COM_traj_right_timeseries = timeseries(COM_traj_right, t);