function zNext = plant(arm)
% This function solves the nonlinear equations of motion of an arm model
% (over a single time step) using MATLAB's variable-step-size numerical
% integrator 'ode45'. It updates the arm properties and outputs the
% augmented state vector (as required for this function to be used in the
% unscented Kalman filter for state estimation).

% ___________                     _____________________________
% |         |              u*     |                           |   x
% | Control |_____________________| Plant                     |____
% |_________|              |      |   solve the differential  |
%       |                  |      |   equation of motion for  |
%       |                  |      |   the model given by      |
%       |                  |      |   x_dot = f(x,u)          |
%       |                  |      |___________________________|
%       |           _______|_____            | 
%       |           |           |            | y
%       |___________| Estimator |____________|
%         x_est     |___________|

% solve the equations of motion using ode45
[~, xTraj] = ode45(@(t,x) dynamics(arm,x,u), [0,arm.Ts], arm.x.val);

% save the integrated result as new state of the arm
arm.x.val = xTraj(end,:)';

% save new arm state in the augmented state vector
nStates = length( arm.x.min );
xNext = arm.x.val;
xNext(1:nStates) = arm.x.val;

% time-shift remainder of augmented state vector
nDelSteps = floor(arm.Td/arm.Ts + 1);
Mprop = diag(ones((nStates)*nDelSteps,1),-(nStates)); % time-shift matrix
xNext = xNext + Mprop*xNext;

if ~model
    xNext(1:nStates) = xNext(1:nStates) + ...
        sqrt(diag(params.Q)).*randn(nStates,1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function currently not working. My error message:
% Error using  * 
% Inner matrix dimensions must agree.
% 
% Error in plant (line 34)
% xNext = xNext + Mprop*xNext;
% 
% Error in main_workspace (line 84)
%         model = plant( model, u_star );
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
