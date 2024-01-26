classdef Traj_Planner
    % Solves for the cubic or quintic trajectory given an trajectory
    % duration, starting and ending position, starting and ending velocity,
    % and acceleration for the quintic trajectory.

    methods
        % @params: duration     --> Time interpolation              (Number)
        % @params: start_v      --> Starting velocity               (Number)
        % @params: end_v        --> Ending velocity                 (Number)
        % @params: start_pos    --> starting position               (Number)
        % @params: end_pos      --> Ending position                 (Number)

        % all values are numerical values of dimension 1x1
        % @returns: matrix with dimension 4x1                   (4x1 matrix)
        function output = cubic_traj(self,duration, start_v, end_v, start_pos, end_pos)
            % solves for a cubic order trajectory between two via-pts
            
            % generate 4x4 matrix based on duration
            t_matrix = [1, 0,         0,                 0;
                        0, 1,         0,                 0;
                        1, duration,  duration*duration, duration*duration*duration;
                        0, 1,         2*duration         3*duration*duration];

            % multiply inverse by 1x4
            % this is funny matlab syntax for specifying that you want to
            % take the inverse of the matrix on the left hand size and then
            % multiply by the matrix on the right hand size

            % inv(A) * b = A \ b
            % returns 4x1 matrix
            output = t_matrix \ transpose([start_pos, start_v, end_pos, end_v]);
    
        end

        % @params: duration     --> Time required for interpolation (Number)
        % @params: start_v      --> Starting velocity              (Number)
        % @params: end_v        --> Ending velocity                (Number)
        % @params: start_pos    --> starting position              (Number)
        % @params: end_pos      --> Ending position                (Number)
        % @params: start_a      --> Starting acceleration          (Number)
        % @params: end_a        --> Ending acceleration            (Number)

        % all values are numerical values of dimension 1x1
        % @returns: 6x1 matrix                                   (6x1 matrix)
        % containing the coefficients ai, i=0,1,2,3,4,5
        function output = quintic_traj(self,tf,start_v, end_v, start_pos, end_pos, start_a, end_a)
            % solves for a quintic order trajectory between two via-pts
            
            ti = 0; % I am leaving ti because I like the formatting
            % generate 6x6 matrix for time

            A = [   1, ti, (ti^2), (ti^3),   (ti^4),    (ti^5);
                    0, 1,  (2*ti), (3*ti^2), (4*ti^3),  (5*ti^4);
                    0, 0,   2,     (6*ti),   (12*ti^2), (20*ti^3);
                    1, tf, (tf^2), (tf^3),   (tf^4),    (tf^5);  
                    0, 1,  (2*tf), (3*tf^2), (4*tf^3),  (5*tf^4);
                    0, 0,   2,     (6*tf),   (12*tf^2), (20*tf^3);
            ];

            b = transpose([start_pos, start_v, start_a, end_pos, end_v, end_a]);

            % multiply inverse by 1x4
            % this is funny matlab syntax for specifying that you want to
            % take the inverse of the matrix on the left hand size and then
            % multiply by the matrix on the right hand size

            % inv(A) * b = A \ b
            output = A \ b; % returns 6x1 array
        end

    end

end