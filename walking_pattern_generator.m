g = 9.81;  % [m/s2]
zc = 0.8; % [m]
Tsup = 0.8;  % [s]
Ts = 0.001;  % [s]

Tc = sqrt(zc / g);
C = cosh(Tsup / Tc);
S = sinh(Tsup / Tc);

a = 10;
b = 1;

D = (a * (C-1)^2) + (b * (S / Tc)^2);

coeff1 = a * (C - 1) / D;
coeff2 = b * S / (Tc * D);

step_x = 0.3; % [m]
step_y = 0.5; % [m]

px_init = 0.0;
py_init = 0.0;

t = 0:Ts:Tsup;

n_steps = 10;

px = zeros(1, n_steps);
py = zeros(1, n_steps);
px_mod = zeros(1, n_steps);
py_mod = zeros(1, n_steps);

x_bar = zeros(1, n_steps);
y_bar = zeros(1, n_steps);
vx_bar = zeros(1, n_steps);
vy_bar = zeros(1, n_steps);

x_target = zeros(1, n_steps);
y_target = zeros(1, n_steps);
xdot_target = zeros(1, n_steps);
ydot_target = zeros(1, n_steps);

x = zeros(1, size(t, 2));
y = zeros(1, size(t, 2));
xdot = zeros(1, size(t, 2));
ydot = zeros(1, size(t, 2));

xdot(1) = 0.4;
ydot(1) = 0.05;

px(1) = px_init;
py(1) = py_init;

for i = 1:n_steps

    if (i > 1)
        x(1) = x(end);
        y(1) = y(end);
        xdot(1) = xdot(end);
        ydot(1) = ydot(end);
        px(i) = px(i-1) + px_mod(i-1);
        py(i) = py(i-1) + py_mod(i-1);
    end

    % Simulate LIPM model
    for j = 2:size(t, 2)
        xdot(j) = xdot(j-1) + Ts * ((zc / g) * (x(j-1) - px(i)));
        ydot(j) = ydot(j-1) + Ts * ((zc / g) * (y(j-1) - py(i)));
    
        x(j) = x(j-1) + Ts * xdot(j-1);
        y(j) = y(j-1) + Ts * ydot(j-1);
    end

    % Next foot placement
    px(i) = px(i) + step_x;
    py(i) = py(i) - (-1)^(i)*step_y;

    % Set walk primitive
    x_bar(i) = step_x * 0.5;
    y_bar(i) = ((-1)^(i))* step_y * 0.5;

    vx_bar(i) = x_bar(i) * (C + 1) / (Tc * S);
    vy_bar(i) = y_bar(i) * (C - 1) / (Tc * S);

    % Calculate target state
    x_target(i) = px(i) + x_bar(i);
    xdot_target(i) = vx_bar(i);
    y_target(i) = py(i) + y_bar(i);
    ydot_target(i) = vy_bar(i);

    % Modified foot placement calculation
    px_mod(i) = -( coeff1 * ( x_target(i) - (C * x(1)) - (Tc * S * xdot(1)) ) ) - ( coeff2 * ( xdot_target(i) - (S * x(1) / Tc) - (C * xdot(1)) ) );
    py_mod(i) = -( coeff1 * ( y_target(i) - (C * y(1)) - (Tc * S * ydot(1)) ) ) - ( coeff2 * ( ydot_target(i) - (S * y(1) / Tc) - (C * ydot(1)) ) );
end

disp(px)
disp(py)
disp(px_mod)
disp(py_mod)
plot(px,py,'-o')