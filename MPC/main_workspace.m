clear; clc; close all;
%%% This script computes the workspace for the simulated arm

%% Step 1: Setup arm and MPC toolbox
% Initialize MPT3 toolbox
addpath( genpath([pwd '/tbxmanager']));

% Subject characteristics
subj.M = 70;    % kg
subj.H = 1.80;  % meters
subj.hand = 'right';

% Define a model.  We'll start with the 2 degree of freedom planar model
model = arm_2DOF(subj);

% Initially, the model will be resting at 0 degrees of shoulder horizontal
% rotation and 0 degrees of elbow flexion.
model.q = model.thLim(:,2);
draw(model);

histories.u = [];
histories.q = model.q;
histories.x = fwdKin( model, model.q );

% ref = [ 30; 30; 0; 0 ] * pi/180;
ref = model.thLim(:,2);
% Perform a reach:
for i = 1:200
    
    % Compute the optimal control value
    u_star = control( model, zeros(2,1), ref, 'joint' );
    
    % Implement the optimal torques on the model.
    model = plant( model, u_star );
    
    % Sense the resulting sensory outputs and estimate the next state.

    draw( model);
    display(['Time = ' num2str(i*model.Ts) 'sec'])
    model.q - histories.q(:,end)
    
    histories.u = [ histories.u, u_star ];
    histories.q = [ histories.q, model.q ];
    histories.x = [ histories.x, fwdKin( model, model.q )];
end

figure
subplot(3,1,1)
    plot( model.Ts*(1:length(histories.u(1,:))), histories.u(1,:), ...
          model.Ts*(1:length(histories.u(1,:))), histories.u(2,:))
    ylabel 'Optimal joint torques, N-m'
    box off
    
subplot(3,1,2)
    plot( model.Ts*(0:length(histories.u(1,:))), histories.q(1,:)*180/pi,...
          model.Ts*(0:length(histories.u(1,:))), histories.q(2,:)*180/pi )
    ylabel 'Joint angle trajecotires, degrees'
    box off
subplot(3,1,3)
    plot( model.Ts*(0:length(histories.u(1,:))), histories.x(1,:),...
          model.Ts*(0:length(histories.u(1,:))), histories.x(2,:),...
          model.Ts*(0:length(histories.u(1,:))), histories.x(3,:),...
          model.Ts*(0:length(histories.u(1,:))), histories.x(4,:) )
    ylabel 'Hand trajectory, m'
    xlabel 'Time (sec)'
    box off