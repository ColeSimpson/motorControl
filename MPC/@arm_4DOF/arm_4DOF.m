% This class captures the attributes, kinematics, and dynamics of a human
% arm modeled as an RR robot restricted to move in the plane of the
% shoulder.
classdef arm_4DOF < handle
    
    % properties that, once set in constructor, remain constant
    properties (GetAccess=public, SetAccess=private)
        
        jDOF = 4;         % joint-space degrees of freedom (shoulder & 
                          % elbow angle)
        tDOF = 3;         % task-space degrees of freedom (x & y, not theta)
        shld = [0;0;0];   % position of shoulder, in task coordinates [m]
        B = [0.25 .* ones(3,3); % damping matrix, Crevecouer 2013 [Nms/rad]
        B = B + eye(3).*2.75;
        
        hand;    % handedness [right or left]
        m1;      % upperarm mass, Winter [kg]
        m2;      % forearm mass, Winter [kg]
        l1;      % upperarm length, Winter [m]
        l2;      % forearm length, Winter [m]
        s1;      % shoulder to upperarm COM, Winter [m]
        s2;      % elbow to forearm COM, Winter [m]
        r1;      % upperarm radius of gyration (proximal), Winter [m]
        r2;      % forearm radius of gyration (proximal), Winter [m]
        I1;      % upperarm moment of inertia about shoulder, Winter [kg-m^2]
        I2;      % forearm moment of inertia about elbow, Winter [kg-m^2]
        thLim;   % joint angle limits [rad]
        torqLim; % joint torque limits [Nm]
        
    end
    
    % properties that are initialized in constructor but change over the
    % course of simulation
    properties (GetAccess=public, SetAccess=public)
        
        q;    % state, in joint coordinates [rad,rad/s]
        x;    % state, in task coordinates [m,m/s]
        elbw; % position of elbow, in task coordinates [m]
        inWS; % 1 = in workspace, 0 = outside workspace
    
    end
    
    % methods that can be called from outside of the class
    methods (Access=public)
        
        % constructor
        function arm = arm_4DOF(subj, movt)
            if  nargin > 0
                
                % set constant properties
                arm.hand = subj.hand;
                arm.m1 = 0.028*subj.M;
                arm.m2 = 0.022*subj.M;
                arm.l1 = 0.188*subj.H;
                arm.l2 = 0.253*subj.H;
                arm.s1 = 0.436*arm.l1;
                arm.s2 = 0.682*arm.l2;
                arm.r1 = 0.542*arm.l1;
                arm.r2 = 0.827*arm.l2;
                arm.I1 = arm.m1*arm.r1^2;
                arm.I2 = arm.m2*arm.r2^2;
                arm.thLim = [subj.th1Min, subj.th1Max;
                             subj.th2Min, subj.th2Max;
                             subj.th3Min, subj.th3Max;
                             subj.th4Min, subj.th4Max ] * (pi/180);
                arm.torqLim = [subj.torq1Min, subj.torq1Max;
                               subj.torq2Min, subj.torq2Max;
                               subj.torq3Min, subj.torq3Max;
                               subj.torq4Min, subj.torq4Max ];
                
                % initialize dynamic properties
                arm.x = [movt.p_i;movt.v_i];
                [arm.q, arm.elbw, arm.inWS] = arm.invKin();
                
            end
        end
        
        % function prototypes
        flag = withinLimits(arm, q)
        [x, elbw, reachable] = fwdKin(arm, q)
        [q, elbw, reachable] = invKin(arm, x)
        f = dynamics(arm, u, ctrlSpace)
        M = draw(arm)
        
    end
    
    % methods that are only available to other class/subclass methods
    methods (Access=protected)
        
        J = jacobian(arm, q)
        J_dot = jacobianDeriv(arm, q)
        
    end
    
end