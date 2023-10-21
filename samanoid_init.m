g_vector = [0 0 -9.81];  % [m/s^2]

origin_com_offset = 0.073481;  % [m]
COM_init = [0.13, 0.25, 1.1037];

torso_z_init = COM_init(3) - origin_com_offset;  % [m]

theta_guess = deg2rad([0; 20; -40; 20; 0]);
