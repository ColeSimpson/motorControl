% This function returns the derivative of the Jacobian for the arm in a
% given state, taking handedness into account. If no state is specified as
% input, the function computes the derivative for the current state of the
% 'arm' object.
function J_dot = jacobianDeriv(arm, x)

% if no state specified, use current state of arm
if nargin < 2
    x = arm.x.val;
end

% Right now I only have the parameters for a right arm
J_dot(1,1) = arm.l4*(cos(x(4))*x(8)*(cos(x(3))*sin(x(1)) - ...
    cos(x(1))*sin(x(2))*sin(x(3))) - sin(x(4))*(sin(x(1))*sin(x(3))*x(7)...
    - cos(x(1))*cos(x(3))*x(5) + cos(x(1))*cos(x(2))*sin(x(3))*x(6) + ...
    cos(x(1))*cos(x(3))*sin(x(2))*x(7) - ...
    sin(x(1))*sin(x(2))*sin(x(3))*x(5)) + ...
    cos(x(2))*cos(x(4))*sin(x(1))*x(5) + ...
    cos(x(1))*cos(x(4))*sin(x(2))*x(6) + ...
    cos(x(1))*cos(x(2))*sin(x(4))*x(8)) + ...
    arm.l3*cos(x(2))*sin(x(1))*x(5) + arm.l3*cos(x(1))*sin(x(2))*x(6);
J_dot(1,2) = sin(x(1))*(arm.l3*cos(x(2))*x(6) + ...
    arm.l4*cos(x(2))*cos(x(4))*x(6) - arm.l4*sin(x(2))*sin(x(4))*x(8) - ...
    arm.l4*cos(x(2))*cos(x(3))*sin(x(4))*x(7) - ...
    arm.l4*cos(x(2))*cos(x(4))*sin(x(3))*x(8) + ...
    arm.l4*sin(x(2))*sin(x(3))*sin(x(4))*x(6)) + ...
    cos(x(1))*x(5)*(arm.l3*sin(x(2)) + arm.l4*cos(x(4))*sin(x(2)) - ...
    arm.l4*cos(x(2))*sin(x(3))*sin(x(4)));
J_dot(1,3) = arm.l4*cos(x(4))*x(8)*(cos(x(1))*sin(x(3)) - ...
    cos(x(3))*sin(x(1))*sin(x(2))) - ...
    arm.l4*sin(x(4))*(sin(x(1))*sin(x(3))*x(5) - ...
    cos(x(1))*cos(x(3))*x(7) + cos(x(1))*cos(x(3))*sin(x(2))*x(5) + ...
    cos(x(2))*cos(x(3))*sin(x(1))*x(6) - ...
    sin(x(1))*sin(x(2))*sin(x(3))*x(7));
J_dot(1,4) = arm.l4*(sin(x(4))*x(8)*(cos(x(1))*cos(x(3)) + ...
    sin(x(1))*sin(x(2))*sin(x(3))) - ...
    cos(x(4))*(cos(x(1))*sin(x(2))*sin(x(3))*x(5) - ...
    cos(x(1))*sin(x(3))*x(7) - cos(x(3))*sin(x(1))*x(5) + ...
    cos(x(2))*sin(x(1))*sin(x(3))*x(6) + ...
    cos(x(3))*sin(x(1))*sin(x(2))*x(7)) + ...
    cos(x(1))*cos(x(2))*sin(x(4))*x(5) + ...
    cos(x(2))*cos(x(4))*sin(x(1))*x(8) - ...
    sin(x(1))*sin(x(2))*sin(x(4))*x(6));


J_dot(2,1) = 0;
J_dot(2,2) = arm.l4*(cos(x(2))*sin(x(3))*sin(x(4))*x(6) - ...
    cos(x(2))*sin(x(4))*x(8) - cos(x(4))*sin(x(2))*x(6) + ...
    cos(x(3))*sin(x(2))*sin(x(4))*x(7) + ...
    cos(x(4))*sin(x(2))*sin(x(3))*x(8)) - arm.l3*sin(x(2))*x(6);
J_dot(2,3) = arm.l4*cos(x(3))*sin(x(2))*sin(x(4))*x(6) - ...
    arm.l4*cos(x(2))*cos(x(3))*cos(x(4))*x(8) + ...
    arm.l4*cos(x(2))*sin(x(3))*sin(x(4))*x(7);
J_dot(2,4) = -arm.l4*(cos(x(2))*sin(x(4))*x(6) + ...
    cos(x(4))*sin(x(2))*x(8) + cos(x(2))*cos(x(3))*cos(x(4))*x(7) - ...
    cos(x(4))*sin(x(2))*sin(x(3))*x(6) - ...
    cos(x(2))*sin(x(3))*sin(x(4))*x(8));


J_dot(3,1) = arm.l4*(sin(x(4))*(cos(x(1))*sin(x(2))*sin(x(3))*x(5) - ...
    cos(x(1))*sin(x(3))*x(7) - cos(x(3))*sin(x(1))*x(5) + ...
    cos(x(2))*sin(x(1))*sin(x(3))*x(6) + ...
    cos(x(3))*sin(x(1))*sin(x(2))*x(7)) + ...
    cos(x(4))*x(8)*(cos(x(1))*cos(x(3)) + ...
    sin(x(1))*sin(x(2))*sin(x(3))) + ...
    cos(x(1))*cos(x(2))*cos(x(4))*x(5) - ...
    cos(x(4))*sin(x(1))*sin(x(2))*x(6) - ...
    cos(x(2))*sin(x(1))*sin(x(4))*x(8)) + ...
    arm.l3*cos(x(1))*cos(x(2))*x(5) - arm.l3*sin(x(1))*sin(x(2))*x(6);
J_dot(3,2) = cos(x(1))*(arm.l3*cos(x(2))*x(6) + ...
    arm.l4*cos(x(2))*cos(x(4))*x(6) - arm.l4*sin(x(2))*sin(x(4))*x(8) - ...
    arm.l4*cos(x(2))*cos(x(3))*sin(x(4))*x(7) - ...
    arm.l4*cos(x(2))*cos(x(4))*sin(x(3))*x(8) + ...
    arm.l4*sin(x(2))*sin(x(3))*sin(x(4))*x(6)) - ...
    sin(x(1))*x(5)*(arm.l3*sin(x(2)) + arm.l4*cos(x(4))*sin(x(2)) - ...
    arm.l4*cos(x(2))*sin(x(3))*sin(x(4)));
J_dot(3,3) = -arm.l4*sin(x(4))*(cos(x(1))*sin(x(3))*x(5) + ...
    cos(x(3))*sin(x(1))*x(7) + cos(x(1))*cos(x(2))*cos(x(3))*x(6) - ...
    cos(x(3))*sin(x(1))*sin(x(2))*x(5) - ...
    cos(x(1))*sin(x(2))*sin(x(3))*x(7)) - ...
    arm.l4*cos(x(4))*x(8)*(sin(x(1))*sin(x(3)) + ...
    cos(x(1))*cos(x(3))*sin(x(2)));
J_dot(3,4) = -arm.l4*(cos(x(4))*(sin(x(1))*sin(x(3))*x(7) - ...
    cos(x(1))*cos(x(3))*x(5) + cos(x(1))*cos(x(2))*sin(x(3))*x(6) + ...
    cos(x(1))*cos(x(3))*sin(x(2))*x(7) - ...
    sin(x(1))*sin(x(2))*sin(x(3))*x(5)) + ...
    sin(x(4))*x(8)*(cos(x(3))*sin(x(1)) - ...
    cos(x(1))*sin(x(2))*sin(x(3))) - ...
    cos(x(1))*cos(x(2))*cos(x(4))*x(8) + ...
    cos(x(2))*sin(x(1))*sin(x(4))*x(5) + ...
    cos(x(1))*sin(x(2))*sin(x(4))*x(6));

if strcmp(arm.hand,'left')
    warning('Parameters are currently only defined for a right hand.')
end

% if strcmp(arm.hand,'right')
%     J_dot = [(-arm.l1*cos(q(1))*q(3) - arm.l2*cos(q(1)+q(2))*(q(3)+q(4))) -arm.l2*cos(q(1)+q(2))*(q(3)+q(4));
%              (-arm.l1*sin(q(1))*q(3) - arm.l2*sin(q(1)+q(2))*(q(3)+q(4))) -arm.l2*sin(q(1)+q(2))*(q(3)+q(4))];
% else
%     J_dot = [( arm.l1*cos(q(1))*q(3) + arm.l2*cos(q(1)+q(2))*(q(3)+q(4)))  arm.l2*cos(q(1)+q(2))*(q(3)+q(4));
%              (-arm.l1*sin(q(1))*q(3) - arm.l2*sin(q(1)+q(2))*(q(3)+q(4))) -arm.l2*sin(q(1)+q(2))*(q(3)+q(4))];
% end

end