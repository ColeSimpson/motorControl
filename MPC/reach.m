% close all
% clear
% clc
% 
% % add folders to path
% addpath(genpath([pwd '/include']));
% 
% % define subject
% subj.hand = 'right'; % hand being tested
% subj.M = 70;         % mass [kg]
% subj.H = 1.80;       % height [meters]
% 
% 
% % define subject's physical arm & internal arm model (mental)
% arm = arm_2DOF(subj);
% intModel = arm_2DOF(subj);
% 
% 
% % extract arm parameters
% nInputs = length(arm.u.val);
% nJoints = length(arm.q.val);
% 
% 
% 
% % define movement parameters
% T = 1;                       % total time to simulate [sec]
% movt.t = 0:arm.Ts:T;         % time vector [sec]
% r = 0.35;                    % reach distance [m]
% th = [0];            % reach angles [deg]
% origin1 = [-0.15;0.3;0];     % origin 1 (arbitrary) [m]
% origin2 = [-0.18;0.56;0];    % origin 2, to match (Beer, 2000) [m]
% origin3 = [-0.15;0.6;0];     % origin 3 (less arbitrary) [m]
% p_i = origin1;               % initial position [m]
% v_i = [0;0;0];               % initial velocity [m/s]
% y_i = [p_i;v_i];             % initial state, in Cartesian coordinates [m,m/s]
% [x_i,~,~] = arm.invKin(y_i); % initial state, in joint coordinates [rad,rad/s]
% 
% 
% movt.space = 'task';         % space in which to track reference ('joint' or 'task')
%    
%     
% for i = 1:length(th)
% 
%     % define movement reference trajectory
%     p_f = p_i + r*[cosd(th(i));sind(th(i));0]; % desired end position [m]
%     v_f = [0;0;0];                             % desired end velocity [m/s]
%     y_f = [p_f;v_f];                           % desired end state, in Cartesian coordinates [m,m/s]
%     [x_f,~,~] = arm.invKin(y_f);               % desired end state, in joint coordinates [rad,rad/s]
%     switch movt.space
%         case 'joint'
%             movt.ref = repmat(x_f,1,length(movt.t)); % joint-space reference to track [rad,rad/s]
%         case 'task'
%             movt.ref = repmat(y_f,1,length(movt.t)); % task-space reference to track [m,m/s]
%         otherwise
%             movt.space = 'task';
%             movt.ref = repmat(y_f,1,length(movt.t)); % task space by default
%     end
% 
%     % reset model state variables to match initial conditions for movement
%     % NOTE: internal model's state estimates are grounded by vision (i.e.,
%     % ----  assuming perfect vision, they match the arm's actual state)
%     arm.u.val = zeros(nInputs,1);
%     arm.uReflex = zeros(nJoints,1);
%     arm.x.val = [x_i;zeros(nInputs,1)];
%     arm.q.val = x_i(1:nJoints);
%     arm.q0 = arm.q.val;
%     arm.y.val = arm.fwdKin;
%     nDelay = ceil(arm.Td/arm.Ts);
%     arm.z.val = repmat(arm.x.val, nDelay+1, 1);
%     arm.P = diag(1e-6*ones(length(arm.z.val),1));
% 
%     intModel.u.val = zeros(nInputs,1);
%     intModel.uReflex = zeros(nJoints,1);
%     intModel.x.val = [x_i;zeros(nInputs,1)];
%     intModel.q.val = x_i(1:nJoints);
%     intModel.q0 = intModel.q.val;
%     intModel.y.val = intModel.fwdKin;
%     nDelay = ceil(intModel.Td/intModel.Ts);
%     intModel.z.val = repmat(intModel.x.val, nDelay+1, 1);
%     intModel.P = diag(1e-6*ones(length(intModel.z.val),1));
% 
%     % simulate reach
%     data = simulate(movt, arm, intModel);
%     u = data.uCmd;
%     x = data.xAct;
%     y = data.yAct;
%     
%     
%     
%     
%     
%     
%     %%%%
%     % compute optimal control trajectory (only if enough time has passed)
%     if movt.t(i) ~= 0 && mod(movt.t(i),arm.Tr) == 0
%         [u_optTraj, flag] = control(intModel, movt.ref(:,i), movt.space);
%         if flag
%             warning('Linearization failed.')
%             return
%         end
%     end
%     
%     % set torque to zero if haven't planned that far in advance;
%     % otherwise, grab torque from preplanned trajectory
%     if isempty(u_optTraj)
%         u_opt = zeros(nInputs,1);
%     else
%         u_opt = u_optTraj(:,1);
%     end
%     
%     % actuate arm with optimal control & sense feedback
%     zNext = actuate(arm, u_opt);
%     x_sens = sense(arm, zNext);
%     
%     % estimate current state, storing it in internal model
%     estimate(intModel, u_opt, x_sens);
% 
%     % discard most recently applied control
%     u_optTraj = u_optTraj(:,2:end);
% 
% end




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
histories.u = zeros(length(arm.u.min), 1);
histories.x = arm.x.val;
histories.y = fwdKin( arm );



% Give us a message so we know what's going on
x_diff = diff( histories.x' );

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
    histories.u = [ histories.u, u_opt ];
    histories.x = [ histories.x, arm.x.val ];
    histories.y = [ histories.y, fwdKin( arm )];

    % Let us know how long the simulation is taking and how much things
    % are changing at each step
    disp(['Time = ' num2str(i*arm.Ts) 'sec'])

    % Update simulation time and 
    i = i +1;
    x_diff = diff( histories.x');
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
% %   plot( time, ones(size(time))*arm.x.min(1,1)*180/pi, 'b:', ...
%         time, ones(size(time))*arm.thLim(1,2)*180/pi, 'b:', ...
%         time, ones(size(time))*arm.thLim(2,1)*180/pi, 'r:', ...
%         time, ones(size(time))*arm.thLim(2,2)*180/pi, 'r:')
    ylabel 'Joint angle trajecotires, degrees'
    box off
subplot(3,1,3)
    plot( time, histories.y(1,:), 'b', ...
          time, histories.y(2,:), 'r' )
    ylabel 'Hand trajectory, m'
    xlabel 'Time (sec)'
    box off
    
figure
% P = Polyhedron( histories.y(1:2,:)' );
% plot(P)
plot(histories.y(1,:), histories.y(2,:), 'bx')
[k,area]=boundary(histories.y(1:2,:)', 1);
hold on
plot(histories.y(1,k), histories.y(2,k))
    box off
    axis equal
    title 'Hand workspace'
    xlabel 'x'
    ylabel 'y'
