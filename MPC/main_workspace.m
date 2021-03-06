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

%% Compute workspace:
% This function computes the workspace of the arm by driving the arm arm
% to reach as far as it can in several directions.  We chose to reach
% towards several points located on a circle of radius, r = 3 m, centered
% at the shoulder.

% Specify the radius of the circle designating target locations
r = 20; % m

% Save the locations of the arm and hand throughout the simulation and the
% computed control values
arm = arm_2DOF(subj);
histories.u = zeros(length(arm.u.min), 1);
histories.x = arm.x.val;
histories.y = fwdKin( arm );

for th = ( 0:180:360 )*pi/180
    % Give us a message so we know what's going on
    disp(['Reach direction: theta = ' num2str(th*180/pi) 'Deg'])
    disp('_________________________________')
    
    % Update the position of the reference
    ref = [ r*cos(th); r*sin(th); 0; 0; 0;0 ];

    % Reset the arm so we start from the same initial posture at the
    % beginning of each reach.
    arm = arm_2DOF(subj);
    intModel = arm;

    [data, arm] = reach(arm, intModel, ref);
       
    % Save new control, arm configuration, and hand location values
    histories.u = [ histories.u, data.u ];
    histories.x = [ histories.x, data.x ];
    histories.y = [ histories.y, data.y];

end

%% Plot results
time = linspace( 0, arm.Ts*length(histories.u(1,:)), ...
    length(histories.u(1,:)));


figure
subplot(3,1,1)
    plot( time, histories.u(1,:), 'b', ...
          time, histories.u(2,:), 'r')
      hold on
      plot( time, ones(size(time))*arm.u.min(1), 'b:', ...
            time, ones(size(time))*arm.u.min(2), 'b:', ...
            time, ones(size(time))*arm.u.max(1), 'r:', ...
            time, ones(size(time))*arm.u.max(2), 'r:')
    ylabel 'Optimal joint torques, N-m'
    box off
    
subplot(3,1,2)
    plot( time, histories.x(1,:)*180/pi, 'b', ...
          time, histories.x(2,:)*180/pi, 'r' )
   hold on
  plot( time, ones(size(time))*arm.x.min(1)*180/pi, 'b:', ...
        time, ones(size(time))*arm.x.min(2)*180/pi, 'b:', ...
        time, ones(size(time))*arm.x.min(3)*180/pi, 'r:', ...
        time, ones(size(time))*arm.x.min(4)*180/pi, 'r:')
    ylabel 'Joint angle trajecotires, degrees'
    box off
subplot(3,1,3)
    plot( time, histories.y(1,:), 'b', ...
          time, histories.y(2,:), 'r' )
    ylabel 'Hand trajectory, m'
    xlabel 'Time (sec)'
    box off
    
    
    
figure
plot(histories.y(1,:), histories.y(2,:), 'bx')
[k,area]=boundary(histories.y(1:2,:)', 1);
hold on
plot(histories.y(1,k), histories.y(2,k))
    box off
    axis equal
    title 'Hand workspace'
    xlabel 'x'
    ylabel 'y'
    
disp(['Workspace area is ' num2str(area) ' square meters'])