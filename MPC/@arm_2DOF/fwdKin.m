% This function translates a given arm state from joint space to task
% space, taking joint limits (i.e., can that state be reached?) and
% handedness into account. It assumes that the shoulder is at (0,0), the
% default for any 'arm' object. It also outputs elbow position for
% plotting. If no state is specified as input, the function performs
% forward kinematics on the current state of the 'arm' object.
function [x, elbow, reachable] = fwdKin(arm, q)

% check that given state is within joint limits
if nargin == 2
    reachable = arm.withinLimits(q);
    if ~reachable
        x = NaN;
        elbow = NaN;
        return
    end
% if no state specified, use current arm state, assuming reachable
else
    q = arm.q;
    reachable = 1;
end

% POSITION
if strcmp(arm.hand,'right')
    elbow = arm.shld + [arm.l1*cos(q(1));arm.l1*sin(q(1))];
    x(1:2,:) = elbow + [arm.l2*cos(q(1)+q(2));arm.l2*sin(q(1)+q(2))];
else
    elbow = arm.shld + [arm.l1*cos(pi-q(1));arm.l1*sin(pi-q(1))];
    x(1:2,:) = elbow + [arm.l2*cos(pi-q(1)-q(2));arm.l2*sin(pi-q(1)-q(2))];
end

% VELOCITY
J = arm.jacobian(q);
x(3:4,:) = J*q(3:4);

end