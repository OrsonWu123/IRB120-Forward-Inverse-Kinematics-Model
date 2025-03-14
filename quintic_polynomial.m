function [q,qd,qdd]=quintic_polynomial(beginning,ending,T)

    q_given = [beginning, ending; % position
               0, 0; % varying velocity
               0, 0]'; % varying acceleration

    t_given = [0, T(end)]'; % time

    t = T;% time for interpolation

    polynomial5_interpolation = Polynomial5Interpolation('Polynomial 5', q_given, t_given);
    polynomial5_trajectory = zeros(length(t), 3); %position, velocity, acceleration

    for i = 1:length(t)
        polynomial5_trajectory(i,:) = polynomial5_interpolation.getPosition(t(i));
    end

    q=polynomial5_trajectory(:,1);
    qd=polynomial5_trajectory(:,2);
    qdd=polynomial5_trajectory(:,3);

end
