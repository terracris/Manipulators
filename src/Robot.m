classdef Robot < handle
    
    properties
        myHIDSimplePacketComs;
        pol; 
        GRIPPER_ID = 1962;
        jointSetpointGoal = zeros(1, 3, 'single');
        full_transformation = sym('T', [4 4]);
        intermediate_matrices = sym('I', [4 4 3]);
        jacobian = sym('I', [6 3]);
        ik_theta1;
        ik_theta2;
        ik_theta3;

    end
    
    methods
            
        % The is a shutdown function to clear the HID hardware connection
        function  shutdown(self)
	    % Close the device
            self.myHIDSimplePacketComs.disconnect();
        end
        
        % Create a packet processor for an HID device with USB PID 0x007
        function self = Robot(dev)
            self.myHIDSimplePacketComs=dev; 
            self.pol = java.lang.Boolean(false);
            self.init();  % initializes the transformation matrix of the
                          % Robot property.
        end
        
        % Perform a command cycle. This function will take in a command ID
        % and a list of 32 bit floating point numbers and pass them over the
        % HID interface to the device, it will take the response and parse
        % them back into a list of 32 bit floating point numbers as well
        function com = command(self, idOfCommand, values)
            com= zeros(15, 1, 'single');
            try
                ds = javaArray('java.lang.Double',length(values));
                for i=1:length(values)
                    ds(i)= java.lang.Double(values(i));
                end
                % Default packet size for HID
                intid = java.lang.Integer(idOfCommand);
                self.myHIDSimplePacketComs.writeFloats(intid,  ds);
                ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                for i=1:length(com)
                   com(i)= ret(i).floatValue();
                end
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        function com = read(self, idOfCommand)
            com= zeros(15, 1, 'single');
            try
                % Default packet size for HID
                intid = java.lang.Integer(idOfCommand);
                ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                for i=1:length(com)
                   com(i)= ret(i).floatValue();
                end
            catch exception
              getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        function  write(self, idOfCommand, values)
            try
                % sets the goal joint positions based
                self.setGoalJointPositions(values); 
                
                ds = javaArray('java.lang.Double',length(values));
                for i=1:length(values)
                    ds(i)= java.lang.Double(values(i));
                end
                % Default packet size for HID
                intid = java.lang.Integer(idOfCommand);
                self.myHIDSimplePacketComs.writeFloats(intid,  ds,self.pol);

            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        % Specifies a position to the gripper
        function writeGripper(self, value)
            try
                ds = javaArray('java.lang.Byte',length(1));
                ds(1)= java.lang.Byte(value);
                intid = java.lang.Integer(self.GRIPPER_ID);
                self.myHIDSimplePacketComs.writeBytes(intid, ds, self.pol);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        % Opens the gripper
        function openGripper(self)
            self.writeGripper(180);
        end
        
        % Closes the gripper
        function closeGripper(self)
            self.writeGripper(0);
        end

        % -----------------------  Written by Us  -------------------------

        % -------------------     init functions    --------------------
        
        % function: init()
        % initializes the full matrix transformation matrix with the hard
        % coded valies
        function init(self) 
            syms theta1 theta2 theta3 d1 d2 d3 a1 a2 a3 alpha1 alpha2 alpha3
            dhtable = [theta1 d1 a1 alpha1; (theta2-90) d2 a2 alpha2; (theta3+90) d3 a3 alpha3];

            self.full_transformation = self.dh2fk(dhtable);
            % substitute symbolic matrix with known constants
            self.full_transformation = subs(self.full_transformation, {...
                                                        d1, d2, d3, ...
                                                        a1, a2, a3, ...
                                                        alpha1, alpha2, alpha3}, ...
                                                        {
                                                        95, 0, 0, ...
                                                        0 100,100, ...
                                                        (-90), 0, 0});

            self.ik3001Init();
            self.jacob3001_init();

        end

        function ik3001Init(self)
            syms theta1 theta2 theta3
            syms x y z

            l0 = 55;
            l1 = 40;
            l2 = 100;
            l3 = 100;


            d = power(x.^2 + y.^2, 0.5);

            n = z - l0 - l1; %the distance from joint2 to the end effector along the z axis
            r = power( d.^2+n.^2, 0.5); % shortest distance between the end effector and joint2
            R = acosd( (l2.^2+l3.^2-r.^2) / (2*l2*l3)  );
            K = 90 - R / 2;%asind(l3*sind(R)/r);%
            L = asind(n/r);%atan2d(n, x);
            

            self.ik_theta1 = atan2d(y, x);
            self.ik_theta3 = 90 - R;
            self.ik_theta2 = 90 - K - L;%(R/2) - K + theta3;%
        end 


        function jacob3001_init(self)
            syms theta1 theta2 theta3
            p = self.full_transformation(1:3, 4);
        
            x1 = diff(p(1), theta1);
            x2 = diff(p(1), theta2);
            x3 = diff(p(1), theta3);
            
            y1 = diff(p(2), theta1);
            y2 = diff(p(2), theta2);
            y3 = diff(p(2), theta3);
        
            z1 = diff(p(3), theta1);
            z2 = diff(p(3), theta2);
            z3 = diff(p(3), theta3);
        
            z00 = [0;0;1];
            z01 = self.full_transformation(1:3, 3);
            z02 = z01;  % same because they are parallel
        
            self.jacobian = [x1, x2, x3;
                            y1, y2, y3;
                            z1, z2, z3;
                            z00, z01, z02];
        end

        %----------------- Motion Functions ------------------
        
        % function: goal_js()
        % returns the end-of-motion joint setpoint positions in degrees.
        function jointSetpoints = goal_js(self)
            jointSetpoints = self.jointSetpointGoal;
        end

        % function: setGoalJointPositions()
        % input: 1x3 or 3x1 matrix
        % sets the joint positions
        function setGoalJointPositions(self, val)
            self.jointSetpointGoal = [val(3), val(4), val(5)];
        end

        % function: servo_jp()
        % input: motor_values (1x3 or 3x1 matrix)
        % moves each motor to a specific position with no interpolation
        function servo_jp(self, motor_values)
            % 0 for time since no interpolation, 0 for linear, and motor_values

            values = zeros(15,1,'single');
            values(1:5) = [0; 0; motor_values(1); motor_values(2); motor_values(3)];
            self.write(1848, values)  
        end

        % function: interpolate_jp
        % input: time (1x1), motor_values (1x3 or 3x1 matrix)
        % moves each motor to a specific position with interpolation
        function interpolate_jp(self, ip_time, motor_values)
            % 0 for time since no interpolation, 0 for linear, and motor_values
            values = zeros(15,1,'single');
            values(1:5) = [ip_time; 0; motor_values(1); motor_values(2); motor_values(3)];
            self.write(1848, values)          
        end


        % function: measured_js()
        % input: GETPOS, GETVAL (boolean)
        % output: 2x3 matrix. each column: measured joint position and velocity
        % returns the positions and velocities of the 3 joints if the
        % required booleans are true
        function output = measured_js(self, GETPOS, GETVEL)
            % initialize the ouput array as a 2x3
            output = [0 0 0; 0 0 0];

            % if GETPOS flag is true then read a packet from the robot to
            % get the positions and then assign them to the output matrix
            if (GETPOS == true)
                % read packet from robot
                posPacket = self.read(1910);

                % assign joint positions to the output matrix
                output(1,1) = posPacket(3);
                output(1,2) = posPacket(5);
                output(1,3) = posPacket(7);

            end

            % if the GETVEL flag is true then read a packet from the robot
            % to get the velocities and then assign them to the output
            % matrix
            if (GETVEL == true)
                %read packet from the robot
                velPacket = self.read(1822);

                % assign joint velocities to the output matrix
                output(2,1) = velPacket(3);
                output(2,2) = velPacket(6);
                output(2,3) = velPacket(9);
            end
        end

        % function: setpoint_js()
        % output: 1x3 matrix
        % returns the setpoints for the robot's joint positons 
        function output = setpoint_js(self)
            % initialize the ourput matrix
            output = [0 0 0];

            % read packet from robot
            spPacket = self.read(1910);

            % assign joint setpoints to output matrix
            output(1) = spPacket(2);
            output(2) = spPacket(4);
            output(3) = spPacket(6);
        end

        %-----------------   Forward Kinematics   ------------------

        % function: dh2mat()
        % output: 4x4 matrix of intermediate transformation
        % takes in a 1x4 row from DH parameters, returns trans matrix A
        function mat = dh2mat (self, theta, d, a, alpha)
            mat = [ cosd(theta), -sind(theta)*cosd(alpha), sind(theta)*sind(alpha), a*cosd(theta);
                sind(theta), cosd(theta)*cosd(alpha), -cosd(theta)*sind(alpha), a*sind(theta); 
                0 sind(alpha) cosd(alpha) d; 
                0 0 0 1];
        end

        % function: dh2fk()
        % generates intermediate and final transformation matrix from the
        % base to the end effector
        % output: 4x4 matrix representing the final transformation
        function output = dh2fk(self, dhtable)
            output = sym('T', [4 4]);
            
            for i=1:size(dhtable)
                t = self.dh2mat(dhtable(i,1),dhtable(i,2),dhtable(i,3),dhtable(i,4));
                % store in array of matrices
                if (i == 1)
                    output = t;
                else
                    output = output * t;
                end
                self.intermediate_matrices(:,:,i) = output;
            end
           
            self.full_transformation = output;

        end

        % function: fk3001()
        % takes 1x3 or 3x1 array of joint angles and returns the forward 
        % kinematics of the end effector to the base 
        % output: 4x4 matrix
        function resultant = fk3001(self,angle)
            syms theta1 theta2 theta3
            resultant = subs(self.full_transformation, {theta1, theta2, theta3}, ...
                {angle(1), angle(2), angle(3)});
          
            resultant = eval(resultant);

        end

        % function: measure_cp()
        % output: 4x4 matrix 
        function output = measured_cp (self)
            % Takes data from measured_js() and returns a 4 × 4 
            % homogeneous transformation matrix based upon the current joint positions in degrees.
            syms theta1 theta2 theta3

            current_jp = self.measured_js(true, false);
            
            output = subs(self.full_transformation, {theta1,theta2, theta3}, ...
                {current_jp(1,1), current_jp(1,2), current_jp(1,3)});
            output = eval(output);
        end


        % function: setpoint_cp()
        % output: 4x4 matrix
        function output = setpoint_cp(self)
            % returns homogeneous transformation matrix based on the
            % current joint set points, returns intermediate position
            % if interpolation is being used

            syms theta1 theta2 theta3

            current_jp = self.setpoint_js();
            output = subs(self.full_transformation, {theta1,theta2, theta3}, ...
                {current_jp(1,1), current_jp(1,2), current_jp(1,3)});
            output = eval(output);
        end

        % function: goal_cp()
        % output: 4x4 matrix
        function output = goal_cp(self)
            % returns homogeneous transformation matrix based on the
            % end of motion joint set points
            syms theta1 theta2 theta3

            current_jp = self.goal_js();
            output = subs(self.full_transformation, {theta1,theta2, theta3}, ...
                {current_jp(1), current_jp(2), current_jp(3)});
            output = eval(output);
        end   
           
        % --------------- Inverse Kinematics   --------------- 
        
        % function: ik3001() generates joint positions based on a
        % position
        % output: 1x3 matrix of joint positions
        function q = ik3001(self, xyz)
            syms x y z theta3
            l0 = 55;
            l1 = 40;
            l2 = 100;
            l3 = 100;

            distance = power( power(xyz(1), 2) + power(xyz(2), 2) + power(xyz(3)-l0-l1, 2), .5);

            if (distance > l2 + l3)
                error("too far: out of reachable workspace")
            end

            eTheta1 = eval(subs(self.ik_theta1, [x y], [xyz(1), xyz(2)]));
            eTheta3 = eval(subs(self.ik_theta3, [x y z], [xyz(1), xyz(2), xyz(3)]));

            eTheta2 = eval(subs(self.ik_theta2, [x y z theta3], [xyz(1), xyz(2), xyz(3), eTheta3]));
              
%                 if (eTheta1 > 90 || eTheta1 < -90 ||  eTheta2 > 95 || eTheta2 < -45 || eTheta3 > 48 || eTheta3 < -50)
%                    disp([eTheta1, eTheta2, eTheta3]);
%                    error("angle out of bounds");
%                 end 

             q = [eTheta1, eTheta2, eTheta3];
        end

        % --------------- Trajectory Functions ---------------

        % function: run_trajectory
        % this runs a cubic or quintic polynomical trajectory
        % it returns data such as the velocity of the end effector and
        % of the joints
        % input: trajectory coefficients of cubic or quintic motion,
        % duration of the motion, and model object to plot the arm real
        % time
        % output: data (i-1) x 7 matrix.
        % 
        function run_trajectory(self, traj_coeffs, duration)
            % while loop to calculate current joint poses based on the
            % trajectory coefficients and current time
%             data = zeros(100000000, 7);
            total_time = 0;

            [~, numCols] = size(traj_coeffs);
            tic
            i = 1;

            while (total_time <= duration)
                tf = total_time;
                total_time = total_time + toc;

                tic
                if numCols == 4
                new_durations = [   1;
                                    total_time;
                                    total_time*total_time;
                                    total_time.^3
                                  run  ];
                else
                    new_durations = [ 1;
                                      total_time;
                                      total_time.^2
                                      total_time.^3;
                                      total_time.^4;
                                      total_time.^5
                                    ];

                end
                
                new_sp_x = traj_coeffs(1,:) * new_durations;
                new_sp_y = traj_coeffs(2,:) * new_durations;
                new_sp_z = traj_coeffs(3,:) * new_durations;
                new_sp = [new_sp_x, new_sp_y, new_sp_z];

                new_jp = self.ik3001(new_sp);

                self.servo_jp(new_jp);
                measured = self.measured_js(true, true);
                i = i+1;

%                 %q = [measured(1,1); measured(1,2); measured(1,3)];
%                 q_dot = [measured(2,1); measured(2,2); measured(2,3)];
%                 p_dot = self.fdk3001(q,q_dot);
%                 data(i,:) =  [tf, p_dot(1,:), p_dot(2,:), p_dot(3,:), p_dot(4,:), p_dot(5,:), p_dot(6,:)];
% 
%                 h1 = model.plot_arm(q, p_dot(1:3,:));
%                 pause(0.05);
%                 delete(h1);
            end
%             data = data(1:i, :);
            % commands the robot to go there with servo_jp
            % save time and joint data in a matrix for later
            % return saved data
        end

        % moves the robot through a velocity trajectory.
        % in this motion the robot orients and moves the end effector in the
        % direction of the target position
        %input: r_target: the desired position in task space
        function run_velocity_trajectory(self, r_target)
            measured = self.measured_js(true, false);
            curr_q = measured(1, 1:3); % gets all the angles
    
            current = self.fk3001(curr_q);
            r_curr = current(1:3, 4); % gets the position of the end effector at measured joint variables
    run_velocity_trajectory
            tic
            startTime = 0;
            while(~atPoint(r_curr, r_target))
                delta_r = r_target - r_curr;
        
                unit_r = delta_r / norm(delta_r); % unit vector in direction of target point
                speed = 10;
        
                curr_velocity = speed * unit_r;
        
                q_dot = self.idk(curr_q,curr_velocity);
        
                delta_t = toc - startTime;
                new_jp = curr_q + delta_t*q_dot;
        
                self.servo_jp(new_jp);
        
                current = self.fk3001(curr_q);
                r_curr = current(1:3, 4); % gets the position of the end effector at measured joint variable
            end
        end

        % helper function to determine if the end effector is within a
        % threshold to a certain position.
        % input: current position (1x3 or 3x1) and target position (1x3 or 3x1)
        % output: boolean
        function out = atPoint(self,position, target)
            out = (position(1) == target(1)) & (position(2) == target(2)) & (position(3) == target(3));
        end

        %------------------------- Jacobian Functions -----------------------
        % returns the jacobian for a given joint configuration
        % input: q (1x3 or 3x1)
        % output: 6x3 matrix
        function resultant = jacob3001(self,q)
            syms theta1 theta2 theta3
            
            resultant = subs(self.jacobian, {theta1, theta2, theta3}, ...
                        {q(1), q(2), q(3)});
                  
            resultant = eval(resultant); 
        end
          
        % function that validates the functionality of the jacobian.
        % input: q (1x3 or 3x1)
        % output: boolean
        function out = validateJacob3001(self, q)
            valid = true;
            zeroThresh = 1;
            j = self.jacob3001(q);
            m = [j(1,1) j(1,2) j(1,3); j(2,1) j(2,2) j(2,3); j(3,1) j(3,2) j(3,3)];
            d = det(m);
            for i = 1:6
                if (j(i,1) > zeroThresh)
                    valid = false;
                end
            end
        
            if(d == 0)
                valid = false;
            end
        
            out = valid;
        end
        
        function p_dot = fdk3001(self, q, q_dot)
            % takes current joint angles, vector of joint velocities
            % return 6x1 vector of linear and angular velocities (task-space)
            % q as input to jacob3001() to calc manip jacobian for current config
        
            resultant = self.jacob3001(q);
            p_dot = resultant * q_dot; 
        end 
        
        function readPositions(self)
            while(true)
                self.measured_js(true, false)
            end
        end

        function unsorted_centroids = getUnsortedCentroids(self, centroids)
            num_centroids = size(centroids,1);
            unsorted_centroids = zeros(num_centroids,2); % stores pixel x y of centroids in bounded area
            index = 1;
            % for every centroid, if between the boundaries, add to list
            for i=1:num_centroids
                if centroids(i,1) > self.unsorted_low_x && centroids(i,1) < self.unsorted_high_x
                    if centroids(i,2) > self.unsorted_low_y && centroids(i,2) < self.unsorted_high_y
                        unsorted_centroids(index,1) = centroids(i,1);
                        unsorted_centroids(index,2) = centroids(i,2);
                        index = index+1;
                    end
                end
            end
            unsorted_centroids = unsorted_centroids(1:index-1,:);
        end
    
    
    %----------- Final Project State Machine Functions -----------
        % input: string with color, must be valid color
        % output: 1x3 vector of xyz coordinates in robot frame
        function location = ballLocation(self, color)
            % switch case for each color
            % determine a specific location or area each ball should go
    
            location = 0
        end
        
        % input: 1x3 vector of xyz coordinates in robot frame
        function moveToBall(self, tp, current_location, location)
            % position above the orb (up by certain z value)
            % convert object's location to IK joint angles
%             new_jp = self.ik3001(location)
            % move with trajectory planning, plug in joint angles
%             duration = 1;
%             traj_coeffs(1,:) = tp.quintic_traj(duration, 0, 0, current_location(1,1), location(1, 1), 0, 0);
%             traj_coeffs(2,:) = tp.quintic_traj(duration, 0, 0, current_location(1,2), location(1, 2), 0, 0);
%             traj_coeffs(3,:) = tp.quintic_traj(duration, 0, 0, current_location(1,3), location(1, 3), 0, 0);
%         
%             self.run_trajectory(traj_coeffs, duration, model);
%             % move straight down onto orb
%              duration = 1;

%              angles = robot.ik3001(centroidsWorld(1, :));
%              %robot.servo_jp(angles)
%              current_location_angs = robot.measured_js(true, false);
%              cL = robot.fk3001(current_location_angs(1,:));
%              %xyz of current location
%              current_location = [cL(1,4), cL(2,4), cL(3,4)];
             offset = 0;
             duration = 2;

             x_offset = 5;
             y_offset = 0;%offset *  sind(atan2d(location(2), location(1)))+2;
    
             if (current_location(2) < 0)
                 offset = 15;
                 x_offset = 0;
                 y_offset = -5;%offset *  sind(atan2d(location(2), location(1)))+2;
    
             end
    
%              location = [centroidsWorld(1,1),centroidsWorld(1,2),centroidsWorld(1,3)]
    
             traj_coeffs(1,:) = tp.quintic_traj(duration, 0, 0, current_location(1), location(1) + x_offset, 0, 0);
             traj_coeffs(2,:) = tp.quintic_traj(duration, 0, 0, current_location(2), location(2) + y_offset, 0, 0);
             traj_coeffs(3,:) = tp.quintic_traj(duration, 0, 0, current_location(3), location(3) + 40, 0, 0);
             
             self.run_trajectory(traj_coeffs, duration);
             pause(.5)
    
             current_location_angs = self.measured_js(true, false);
             cL = self.fk3001(current_location_angs(1,:));
             current_location = [cL(1,4), cL(2,4), cL(3,4)];
    
             traj_coeffs(1,:) = tp.quintic_traj(duration, 0, 0, current_location(1), location(1) + x_offset, 0, 0);
             traj_coeffs(2,:) = tp.quintic_traj(duration, 0, 0, current_location(2), location(2) + y_offset, 0, 0);
             traj_coeffs(3,:) = tp.quintic_traj(duration, 0, 0, current_location(3), location(3), 0, 0);
             
             self.run_trajectory(traj_coeffs, duration);
    
             pause(1);
             self.closeGripper();
             pause(.5);




        end

        % input: 1x3 vector of xyz coordinates in robot frame
        function moveToGoal(self, tp, current_location, location)
            % convert goal location to IK joint angles
            % move with trajectory planning, plug in joint angles
            duration = 1;
            traj_coeffs(1,:) = tp.quintic_traj(duration, 0, 0, current_location(1,1), location(1, 1), 0, 0);
            traj_coeffs(2,:) = tp.quintic_traj(duration, 0, 0, current_location(1,2), location(1, 2), 0, 0);
            traj_coeffs(3,:) = tp.quintic_traj(duration, 0, 0, current_location(1,3), location(1, 3), 0, 0);
        
            self.run_trajectory(traj_coeffs, duration, model);

            % move down to place it?
        end
    
end
    
    methods(Static)

        function plot = plot3DMat (matOf3DPoints, velocities)
            x = matOf3DPoints(:,1);
            y = matOf3DPoints(:,2);
            z = matOf3DPoints(:,3);                          

            hold on
            view(3);

            plot = plot3(x, y, z, '-o');
            xlim([-150 150]);
            ylim([-100 100]);
            zlim([0 300]);

            % velocity vector
            quiver3(matOf3DPoints(4,1), matOf3DPoints(4,2),matOf3DPoints(4,3), ...
                velocities(1), velocities(2), velocities(3))

            xlabel("x distance (mm)");
            ylabel("y distance (mm)");
            zlabel("z distance (mm)");
            title("End Effector Position");
            
            legend; 
            hold off;
            linkdata on
        end 

    end

end

