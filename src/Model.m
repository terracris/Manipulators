classdef Model
    % plots a stick model of the robot in 3D
    
    properties
        robot
        tempPlot
        h1
    end
    
    methods
        % instantiates the Model class
        function self = Model(robot)
            % takes in robot object
            self.robot = robot;
            hold on;
            % todo: ask what this tempPlot is for
            self.tempPlot = plot3([0 0 0 0], [0 3 30 0], [0 0 30 195], '-o');
            
                xlim([-150 150]);
                ylim([-100 100]);
                zlim([0 300]);
                view(3);
%             self.h1 = quiver3(0, 0, 195, 20, 20, 20);
            hold off;

        end

        function h1 = plot_arm (self, angle, velocities)

           T01 = self.robot.intermediate_matrices(:,:, 1);
           T02 = self.robot.intermediate_matrices(:,:, 2);

           syms theta1 theta2 theta3 d1 d2 d3 a1 a2 a3 alpha1 alpha2 alpha3

            % substitute symbolic matrix with known constants
            % intermediate matrixes tell us the location of the joints with
            % respect to the base frame.
            T01 = subs(T01, {                           theta1, ...
                                                        d1, ...
                                                        a1, ...
                                                        alpha1}, ...
                                                        {angle(1),...
                                                        95, ...
                                                        0, ...
                                                        (-90)});
            T01 = eval(T01);

            %-------------------------------------------
            T02 = subs(T02, {                           theta1, theta2, ...
                                                        d1, d2, ...
                                                        a1, a2, ...
                                                        alpha1, alpha2}, ...
                                                        {angle(1), angle(2),...
                                                        95, 0, ...
                                                         0 100, ...
                                                        (-90), 0});
            
            T02 = eval(T02);

            %-----------------------------------------------

            
            T03 = self.robot.fk3001(angle);

            % each row is contains the coordinates of each joint 
            % with respect to the base frame 

            %                x         y         z
            positions = [ 0,        0,        0;
                          T01(1,4), T01(2,4), T01(3,4);
                          T02(1,4), T02(2,4), T02(3,4);
                          T03(1,4), T03(2,4), T03(3,4)];

            
            hold on;

            % store all the points for each axis
            xdata = positions(:,1);
            ydata = positions(:,2);
            zdata = positions(:,3);

            % plot
            set(self.tempPlot, 'XData', xdata, 'YData', ydata, 'ZData', zdata);

            h1 = quiver3(positions(4,1), positions(4,2),positions(4,3), ...
                    velocities(1), velocities(2), velocities(3));
            hold off;

        end



    end
end

