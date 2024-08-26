function pendulum_simulation()
    % Define parameters
    g = 9.81; % gravity
    l = 1; % length of pendulum
    b = 0.5; % damping factor
    omega = 1; % natural frequency

    % Define time span
    tspan = [0 1000];

    % Define initial conditions
    theta0 = 0;
    omega0 = 0;
    y0 = [theta0; omega0];

    % Define range of driving force
    A_range = 0.5:0.01:2.0;

    % Define the Poincare section
    poincare_section = @(t, y) eventfun(t, y, omega); 
    options = odeset('Events', poincare_section, 'RelTol',1e-6,'AbsTol',1e-6);

    figure;
    hold on;

    for A = A_range
        % Define the ODE
        pendulum_ode = @(t, y) [y(2); -b*y(2) - g/l*sin(y(1)) + A*cos(omega*t)];

        % Solve the ODE
        [t, y, te, ye, ie] = ode45(pendulum_ode, tspan, y0, options);

        % Plot the Poincare section
        if ~isempty(ye) && size(ye, 2) >= 2
            plot(A*ones(size(ye, 1), 1), mod(ye(:, 1), 2*pi), '.');
        end
    end

    xlabel('Driving force (A)');
    ylabel('Theta mod 2pi');
    title('Bifurcation diagram');
end

function [value,isterminal,direction] = eventfun(t, y, omega)
    value = mod(t, 2*pi/omega); % crossing when t is a multiple of the period
    isterminal = 0; % don't stop the integration
    direction = 1; % positive direction
end