
classdef Polynomial5Interpolation < handle
    properties
        name = 'cubic interpolation';
        q_via = [];
        t_via = [];
    end

    methods
        function obj = Polynomial5Interpolation(name, q_via, t_via)
            obj.name = name;
            obj.q_via = q_via;
            obj.t_via = t_via;
            if length(q_via(:,1)) ~= length(t_via)
                error('The q_via and t_via must have a same length');
            end
        end

        function [a0, a1, a2, a3, a4, a5] = polynomial(obj, q0, q1, v0, v1, acc0, acc1, t0, t1)%obj q0 q1 0 0 0 0 0 t1
            if abs(t0 - t1) < 1e-6
                error('t0 and t1 must be different');
            end

            T = t1 - t0;
            h = q1 - q0;

            a0 = q0;
            a1 = v0;
            a2 = acc0/2;
            a3 = (20*h - (8*v1 + 12*v0)*T - (3*acc0 - acc1)*T^2) / (2*T^3);
            a4 = (-30*h + (14*v1 + 16*v0)*T + (3*acc0 - 2*acc1)*T^2) / (2*T^4);
            a5 = (12*h - 6*(v1 + v0)*T + (acc1 - acc0)*T^2) / (2*T^5);
        end


        function q = getPosition(obj, t)
            if (t < obj.t_via(1)) || (t > obj.t_via(end))
                error('time wrong!');
            end

            j = find(obj.t_via >= t, 1, 'first'); % find the index of t1
            if j == 1
                i = 1;
                j = 2;
            else
                i = j-1;
            end

            % get given position
            q0 = obj.q_via(i, 1);
            v0 = obj.q_via(i, 2);
            acc0 = obj.q_via(i, 3);
            t0 = obj.t_via(i);

            q1 = obj.q_via(j, 1);
            v1 = obj.q_via(j, 2);
            acc1 = obj.q_via(j, 3);
            t1 = obj.t_via(j);

            [a0, a1, a2, a3, a4, a5] = obj.polynomial(q0, q1, v0, v1, acc0, acc1, t0, t1);

            q(1, 1) = a0 + a1*(t - t0) + a2*(t-t0)^2 + a3*(t - t0)^3 + a4*(t -  t0)^4 + a5*(t - t0)^5; % position
            q(1, 2) = a1 + 2*a2*(t - t0) + 3*a3*(t - t0)^2 + 4*a4*(t - t0)^3 + 5*a5*(t - t0)^4; % velocity
            q(1, 3) = 2*a2 + 6*a3*(t - t0) + 12*a4*(t- t0)^2 + 20*a5*(t - t0)^3; % acceleration
        end
    end
end

%a0 + a1*(t - t0) + a2*(t-t0)^2 + a3*(t - t0)^3 + a4*(t -  t0)^4 + a5*(t - t0)^5;
