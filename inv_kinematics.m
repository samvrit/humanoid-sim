function current_theta = inv_kinematics(x, y, z, theta1_init, theta2_init, theta3_init, theta4_init, theta5_init)
    current_theta = [theta1_init; theta2_init; theta3_init; theta4_init; theta5_init];

    for i = 1:40
        desired_task_space = [x; y; z; 0; 0; 0];

        T = forward_kinematics(current_theta(1), current_theta(2), current_theta(3), current_theta(4), current_theta(5));

        current_position = [T(1:3,4); 0; 0; 0];

        J = jacobian_calc(current_theta(1), current_theta(2), current_theta(3), current_theta(4), current_theta(5));

        delta_theta = J \ (desired_task_space - current_position);

        current_theta = current_theta + delta_theta;
    end
end

function T = forward_kinematics(theta1, theta2, theta3, theta4, theta5)
    T = [cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)),                                                                                                                sin(theta5)*(cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))),                                                                                                              cos(theta5)*(cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))),                                                                                                                                                                                                                               (9*sin(theta2))/20 + (9*cos(theta2)*sin(theta3))/20 + (9*cos(theta3)*sin(theta2))/20 + (cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))*((273*cos(theta5))/1000 + (3*sin(theta5))/20) + 13/100;
         sin(theta4)*(cos(theta1 - pi/2)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*cos(theta1 - pi/2)) - cos(theta4)*(cos(theta2)*cos(theta1 - pi/2)*sin(theta3) + cos(theta3)*cos(theta1 - pi/2)*sin(theta2)), - cos(theta5)*sin(theta1 - pi/2) - sin(theta5)*(cos(theta4)*(cos(theta1 - pi/2)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*cos(theta1 - pi/2)) + sin(theta4)*(cos(theta2)*cos(theta1 - pi/2)*sin(theta3) + cos(theta3)*cos(theta1 - pi/2)*sin(theta2))), sin(theta5)*sin(theta1 - pi/2) - cos(theta5)*(cos(theta4)*(cos(theta1 - pi/2)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*cos(theta1 - pi/2)) + sin(theta4)*(cos(theta2)*cos(theta1 - pi/2)*sin(theta3) + cos(theta3)*cos(theta1 - pi/2)*sin(theta2))),        (9*cos(theta2)*cos(theta1 - pi/2))/20 - sin(theta1 - pi/2)/10 - sin(theta1 - pi/2)*((3*cos(theta5))/20 - (273*sin(theta5))/1000) - (cos(theta4)*(cos(theta1 - pi/2)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*cos(theta1 - pi/2)) + sin(theta4)*(cos(theta2)*cos(theta1 - pi/2)*sin(theta3) + cos(theta3)*cos(theta1 - pi/2)*sin(theta2)))*((273*cos(theta5))/1000 + (3*sin(theta5))/20) - (9*cos(theta1 - pi/2)*sin(theta2)*sin(theta3))/20 + (9*cos(theta2)*cos(theta3)*cos(theta1 - pi/2))/20;
         cos(theta4)*(cos(theta2)*sin(theta3)*sin(theta1 - pi/2) + cos(theta3)*sin(theta2)*sin(theta1 - pi/2)) + sin(theta4)*(cos(theta2)*cos(theta3)*sin(theta1 - pi/2) - sin(theta2)*sin(theta3)*sin(theta1 - pi/2)), - cos(theta5)*cos(theta1 - pi/2) - sin(theta5)*(cos(theta4)*(cos(theta2)*cos(theta3)*sin(theta1 - pi/2) - sin(theta2)*sin(theta3)*sin(theta1 - pi/2)) - sin(theta4)*(cos(theta2)*sin(theta3)*sin(theta1 - pi/2) + cos(theta3)*sin(theta2)*sin(theta1 - pi/2))), cos(theta1 - pi/2)*sin(theta5) - cos(theta5)*(cos(theta4)*(cos(theta2)*cos(theta3)*sin(theta1 - pi/2) - sin(theta2)*sin(theta3)*sin(theta1 - pi/2)) - sin(theta4)*(cos(theta2)*sin(theta3)*sin(theta1 - pi/2) + cos(theta3)*sin(theta2)*sin(theta1 - pi/2))), (9*sin(theta2)*sin(theta3)*sin(theta1 - pi/2))/20 - cos(theta1 - pi/2)*((3*cos(theta5))/20 - (273*sin(theta5))/1000) - (9*cos(theta2)*sin(theta1 - pi/2))/20 - ((273*cos(theta5))/1000 + (3*sin(theta5))/20)*(cos(theta4)*(cos(theta2)*cos(theta3)*sin(theta1 - pi/2) - sin(theta2)*sin(theta3)*sin(theta1 - pi/2)) - sin(theta4)*(cos(theta2)*sin(theta3)*sin(theta1 - pi/2) + cos(theta3)*sin(theta2)*sin(theta1 - pi/2))) - (9*cos(theta2)*cos(theta3)*sin(theta1 - pi/2))/20 - cos(theta1 - pi/2)/10 + 3/40;
                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                              0,                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               1];
end

function J = jacobian_calc(theta1, theta2, theta3, theta4, theta5)
    J = [sin(theta1 - pi/2)/10 - (9*cos(theta2)*cos(theta1 - pi/2))/20 + sin(theta1 - pi/2)*((3*cos(theta5))/20 - (273*sin(theta5))/1000) + (cos(theta4)*(cos(theta1 - pi/2)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*cos(theta1 - pi/2)) + sin(theta4)*(cos(theta2)*cos(theta1 - pi/2)*sin(theta3) + cos(theta3)*cos(theta1 - pi/2)*sin(theta2)))*((273*cos(theta5))/1000 + (3*sin(theta5))/20) + (9*cos(theta1 - pi/2)*sin(theta2)*sin(theta3))/20 - (9*cos(theta2)*cos(theta3)*cos(theta1 - pi/2))/20, sin(theta1 - pi/2)*(cos(theta1 - pi/2)/10 + cos(theta1 - pi/2)*((3*cos(theta5))/20 - (273*sin(theta5))/1000) + (9*cos(theta2)*sin(theta1 - pi/2))/20 + ((273*cos(theta5))/1000 + (3*sin(theta5))/20)*(cos(theta4)*(cos(theta2)*cos(theta3)*sin(theta1 - pi/2) - sin(theta2)*sin(theta3)*sin(theta1 - pi/2)) - sin(theta4)*(cos(theta2)*sin(theta3)*sin(theta1 - pi/2) + cos(theta3)*sin(theta2)*sin(theta1 - pi/2))) + (9*cos(theta2)*cos(theta3)*sin(theta1 - pi/2))/20 - (9*sin(theta2)*sin(theta3)*sin(theta1 - pi/2))/20) - cos(theta1 - pi/2)*(sin(theta1 - pi/2)/10 - (9*cos(theta2)*cos(theta1 - pi/2))/20 + sin(theta1 - pi/2)*((3*cos(theta5))/20 - (273*sin(theta5))/1000) + (cos(theta4)*(cos(theta1 - pi/2)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*cos(theta1 - pi/2)) + sin(theta4)*(cos(theta2)*cos(theta1 - pi/2)*sin(theta3) + cos(theta3)*cos(theta1 - pi/2)*sin(theta2)))*((273*cos(theta5))/1000 + (3*sin(theta5))/20) + (9*cos(theta1 - pi/2)*sin(theta2)*sin(theta3))/20 - (9*cos(theta2)*cos(theta3)*cos(theta1 - pi/2))/20), sin(theta1 - pi/2)*(cos(theta1 - pi/2)/10 + cos(theta1 - pi/2)*((3*cos(theta5))/20 - (273*sin(theta5))/1000) + ((273*cos(theta5))/1000 + (3*sin(theta5))/20)*(cos(theta4)*(cos(theta2)*cos(theta3)*sin(theta1 - pi/2) - sin(theta2)*sin(theta3)*sin(theta1 - pi/2)) - sin(theta4)*(cos(theta2)*sin(theta3)*sin(theta1 - pi/2) + cos(theta3)*sin(theta2)*sin(theta1 - pi/2))) + (9*cos(theta2)*cos(theta3)*sin(theta1 - pi/2))/20 - (9*sin(theta2)*sin(theta3)*sin(theta1 - pi/2))/20) - cos(theta1 - pi/2)*(sin(theta1 - pi/2)/10 + sin(theta1 - pi/2)*((3*cos(theta5))/20 - (273*sin(theta5))/1000) + (cos(theta4)*(cos(theta1 - pi/2)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*cos(theta1 - pi/2)) + sin(theta4)*(cos(theta2)*cos(theta1 - pi/2)*sin(theta3) + cos(theta3)*cos(theta1 - pi/2)*sin(theta2)))*((273*cos(theta5))/1000 + (3*sin(theta5))/20) + (9*cos(theta1 - pi/2)*sin(theta2)*sin(theta3))/20 - (9*cos(theta2)*cos(theta3)*cos(theta1 - pi/2))/20), sin(theta1 - pi/2)*(cos(theta1 - pi/2)/10 + cos(theta1 - pi/2)*((3*cos(theta5))/20 - (273*sin(theta5))/1000) + ((273*cos(theta5))/1000 + (3*sin(theta5))/20)*(cos(theta4)*(cos(theta2)*cos(theta3)*sin(theta1 - pi/2) - sin(theta2)*sin(theta3)*sin(theta1 - pi/2)) - sin(theta4)*(cos(theta2)*sin(theta3)*sin(theta1 - pi/2) + cos(theta3)*sin(theta2)*sin(theta1 - pi/2)))) - cos(theta1 - pi/2)*(sin(theta1 - pi/2)/10 + sin(theta1 - pi/2)*((3*cos(theta5))/20 - (273*sin(theta5))/1000) + (cos(theta4)*(cos(theta1 - pi/2)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*cos(theta1 - pi/2)) + sin(theta4)*(cos(theta2)*cos(theta1 - pi/2)*sin(theta3) + cos(theta3)*cos(theta1 - pi/2)*sin(theta2)))*((273*cos(theta5))/1000 + (3*sin(theta5))/20)), (cos(theta4)*(cos(theta2)*cos(theta1 - pi/2)*sin(theta3) + cos(theta3)*cos(theta1 - pi/2)*sin(theta2)) - sin(theta4)*(cos(theta1 - pi/2)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*cos(theta1 - pi/2)))*(cos(theta1 - pi/2)*((3*cos(theta5))/20 - (273*sin(theta5))/1000) + ((273*cos(theta5))/1000 + (3*sin(theta5))/20)*(cos(theta4)*(cos(theta2)*cos(theta3)*sin(theta1 - pi/2) - sin(theta2)*sin(theta3)*sin(theta1 - pi/2)) - sin(theta4)*(cos(theta2)*sin(theta3)*sin(theta1 - pi/2) + cos(theta3)*sin(theta2)*sin(theta1 - pi/2)))) + (sin(theta1 - pi/2)*((3*cos(theta5))/20 - (273*sin(theta5))/1000) + (cos(theta4)*(cos(theta1 - pi/2)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*cos(theta1 - pi/2)) + sin(theta4)*(cos(theta2)*cos(theta1 - pi/2)*sin(theta3) + cos(theta3)*cos(theta1 - pi/2)*sin(theta2)))*((273*cos(theta5))/1000 + (3*sin(theta5))/20))*(cos(theta4)*(cos(theta2)*sin(theta3)*sin(theta1 - pi/2) + cos(theta3)*sin(theta2)*sin(theta1 - pi/2)) + sin(theta4)*(cos(theta2)*cos(theta3)*sin(theta1 - pi/2) - sin(theta2)*sin(theta3)*sin(theta1 - pi/2)));
                                                                                                                                                                                                                       (9*sin(theta2))/20 + (9*cos(theta2)*sin(theta3))/20 + (9*cos(theta3)*sin(theta2))/20 + (cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))*((273*cos(theta5))/1000 + (3*sin(theta5))/20) + 13/100,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                -cos(theta1 - pi/2)*((9*sin(theta2))/20 + (9*cos(theta2)*sin(theta3))/20 + (9*cos(theta3)*sin(theta2))/20 + (cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))*((273*cos(theta5))/1000 + (3*sin(theta5))/20)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     -cos(theta1 - pi/2)*((9*cos(theta2)*sin(theta3))/20 + (9*cos(theta3)*sin(theta2))/20 + (cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))*((273*cos(theta5))/1000 + (3*sin(theta5))/20)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         -cos(theta1 - pi/2)*(cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))*((273*cos(theta5))/1000 + (3*sin(theta5))/20),                                                                                                                                                                                                                              (cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))*(cos(theta1 - pi/2)*((3*cos(theta5))/20 - (273*sin(theta5))/1000) + ((273*cos(theta5))/1000 + (3*sin(theta5))/20)*(cos(theta4)*(cos(theta2)*cos(theta3)*sin(theta1 - pi/2) - sin(theta2)*sin(theta3)*sin(theta1 - pi/2)) - sin(theta4)*(cos(theta2)*sin(theta3)*sin(theta1 - pi/2) + cos(theta3)*sin(theta2)*sin(theta1 - pi/2)))) + (cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))*((273*cos(theta5))/1000 + (3*sin(theta5))/20)*(cos(theta4)*(cos(theta2)*sin(theta3)*sin(theta1 - pi/2) + cos(theta3)*sin(theta2)*sin(theta1 - pi/2)) + sin(theta4)*(cos(theta2)*cos(theta3)*sin(theta1 - pi/2) - sin(theta2)*sin(theta3)*sin(theta1 - pi/2)));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 sin(theta1 - pi/2)*((9*sin(theta2))/20 + (9*cos(theta2)*sin(theta3))/20 + (9*cos(theta3)*sin(theta2))/20 + (cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))*((273*cos(theta5))/1000 + (3*sin(theta5))/20)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      sin(theta1 - pi/2)*((9*cos(theta2)*sin(theta3))/20 + (9*cos(theta3)*sin(theta2))/20 + (cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))*((273*cos(theta5))/1000 + (3*sin(theta5))/20)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          sin(theta1 - pi/2)*(cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))*((273*cos(theta5))/1000 + (3*sin(theta5))/20),                                                                                                                                                                                                                              (cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))*(cos(theta4)*(cos(theta2)*cos(theta1 - pi/2)*sin(theta3) + cos(theta3)*cos(theta1 - pi/2)*sin(theta2)) - sin(theta4)*(cos(theta1 - pi/2)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*cos(theta1 - pi/2)))*((273*cos(theta5))/1000 + (3*sin(theta5))/20) - (cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))*(sin(theta1 - pi/2)*((3*cos(theta5))/20 - (273*sin(theta5))/1000) + (cos(theta4)*(cos(theta1 - pi/2)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*cos(theta1 - pi/2)) + sin(theta4)*(cos(theta2)*cos(theta1 - pi/2)*sin(theta3) + cos(theta3)*cos(theta1 - pi/2)*sin(theta2)))*((273*cos(theta5))/1000 + (3*sin(theta5))/20));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           -sin(theta1 - pi/2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           -sin(theta1 - pi/2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           -sin(theta1 - pi/2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           sin(theta4)*(cos(theta1 - pi/2)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*cos(theta1 - pi/2)) - cos(theta4)*(cos(theta2)*cos(theta1 - pi/2)*sin(theta3) + cos(theta3)*cos(theta1 - pi/2)*sin(theta2));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           -cos(theta1 - pi/2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           -cos(theta1 - pi/2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           -cos(theta1 - pi/2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           cos(theta4)*(cos(theta2)*sin(theta3)*sin(theta1 - pi/2) + cos(theta3)*sin(theta2)*sin(theta1 - pi/2)) + sin(theta4)*(cos(theta2)*cos(theta3)*sin(theta1 - pi/2) - sin(theta2)*sin(theta3)*sin(theta1 - pi/2))];

end