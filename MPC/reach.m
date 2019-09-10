clear; clc; close all;
set(0,'DefaultFigureWindowStyle','docked')
%%% This script computes the workspace for the simulated arm

%% Setup arm and MPC toolbox
% Add all include folders to the working directory
addpath( genpath([pwd '/include']));

% Subject characteristics
subj.M = 70;    % kg
subj.H = 1.80;  % meters
subj.hand = 'right';
subj.Td = 0;
subj.coupled = false;

% Define a arm.  We'll start with the 2 degree of freedom planar arm
arm = arm_2DOF(subj);
% arm.draw;
intModel = arm_2DOF(subj); % internal model


%% Compute workspace:
% This function computes the workspace of the arm by driving the arm arm
% to reach as far as it can in several directions.  We chose to reach
% towards several points located on a circle of radius, r = 3 m, centered
% at the shoulder.

% Specify the radius of the circle designating target locations
r = 9; % m

% Save the locations of the arm and hand throughout the simulation and the
% computed control values
data.u = zeros(length(arm.u.min), 1);
data.x = arm.x.val;
data.y = fwdKin( arm );



% Give us a message so we know what's going on
x_diff = diff( data.x' );

% Update the position of the reference
ref = [ .3; 0.3; 0; 0 ];
i = 0;

% Reset the arm so we start from the same initial posture at the
% beginning of each reach.
arm = arm_2DOF(subj);


%% Simulate reach.
% This while loop simulates a movement until the arm stops moving (joint
% velocities and accelerations decrease below 1e-2 radians or
% radians/second) as long as the resulting state is within the joint
% limits.  This loop also assumes that the movement will take at least 0.1
% seconds.
while arm.withinLimits && ( max(abs( x_diff(end,:))) > 1e-3  ...
        || i*arm.Ts < 0.1 )

    % Compute the optimal control value
    try
        u_opt = control( arm,  ref, 'cartesian' );
        % Note: The finite differences method used to linearize the
        % dynamics may cause joint limitation violation warnings even when
        % the actual posture satisfies the constraints.
    catch
        % The mismatch between linear and nonlinear arm arms may cause
        % infeasible states which then result in errors when computing the
        % next u_star.  This catch function allows us to bypass this
        % problem and proceed with computing the rest of the workspace.
        warning(['Optimal control value could not be found, ' ...
            'exiting reach simulation.'])
        break
    end

    %%% Implement the optimal torques on the arm.
    % set torque to zero if haven't planned that far in advance;
    % otherwise, grab torque from preplanned trajectory
    if isempty(u_opt)
        u_opt = zeros(size(arm.u.min));
    else
        u_opt = u_opt(:,1);
    end
    zNext = actuate(arm, u_opt);

    % Sense the resulting sensory outputs and estimate the next state.
    x_sens = sense(arm, zNext);

    % Save new control, arm configuration, and hand location values
    data.u = [ data.u, u_opt ];
    data.x = [ data.x, arm.x.val ];
    data.y = [ data.y, fwdKin( arm )];

    % Let us know how long the simulation is taking and how much things
    % are changing at each step
    disp(['Time = ' num2str(i*arm.Ts) 'sec'])

    % Update simulation time and 
    i = i +1;
    x_diff = diff( data.x');
end

    
    
%% Plot results
time = linspace( 0, arm.Ts*length(data.u(1,:)), ...
    length(data.u(1,:)));
figure
subplot(3,1,1)
    plot( time, data.u(1,:), 'b', ...
          time, data.u(2,:), 'r')
      hold on
      plot( time, ones(size(time))*arm.u.min(1), 'b:', ...
            time, ones(size(time))*arm.u.min(2), 'b:', ...
            time, ones(size(time))*arm.u.max(1), 'r:', ...
            time, ones(size(time))*arm.u.max(2), 'r:')
    ylabel 'Optimal joint torques, N-m'
    box off
    
subplot(3,1,2)
    plot( time, data.x(1,:)*180/pi, 'b', ...
          time, data.x(2,:)*180/pi, 'r' )
   hold on
% %   plot( time, ones(size(time))*arm.x.min(1,1)*180/pi, 'b:', ...
%         time, ones(size(time))*arm.thLim(1,2)*180/pi, 'b:', ...
%         time, ones(size(time))*arm.thLim(2,1)*180/pi, 'r:', ...
%         time, ones(size(time))*arm.thLim(2,2)*180/pi, 'r:')
    ylabel 'Joint angle trajecotires, degrees'
    box off
subplot(3,1,3)
    plot( time, data.y(1,:), 'b', ...
          time, data.y(2,:), 'r' )
    ylabel 'Hand trajectory, m'
    xlabel 'Time (sec)'
    box off
    
figure
% P = Polyhedron( histories.y(1:2,:)' );
% plot(P)
plot(data.y(1,:), data.y(2,:), 'bx')
[k,area]=boundary(data.y(1:2,:)', 1);
hold on
plot(data.y(1,k), data.y(2,k))
    box off
    axis equal
    title 'Hand workspace'
    xlabel 'x'
    ylabel 'y'
