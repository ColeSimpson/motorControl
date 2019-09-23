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
r = 2; % m

parfor i = 0:35
    th = i*10*pi/180;
    
    % Give us a message so we know what's going on
    disp(['Reach direction: theta = ' num2str(th*180/pi) 'Deg'])
    disp('_________________________________')
    
    % Update the position of the reference
    ref = [ r*cos(th); r*sin(th); 0; 0; 0;0 ];

    % Reset the arm so we start from the same initial posture at the
    % beginning of each reach.
    arm = arm_4DOF(subj);
    arm.x.val = [0; 0.8; 0; 0.8; 0; 0 ;0; 0; 0; 0 ;0; 0; 0; 0 ;0; 0];
    intModel = arm;

    [data, arm] = reach(arm, intModel, ref, ['results/healthy4dof/healthy' num2str(th*180/pi) 'deg.mat']);       
end
