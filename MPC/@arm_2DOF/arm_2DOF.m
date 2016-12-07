% This class captures the attributes, kinematics, and dynamics of a human
% arm modeled as an RR robot restricted to move in the plane of the
% shoulder. As a handle class, objects of this type are passed by reference
% into functions. That is, the handle is copied but the copy references the
% same underlying data as the original handle.
classdef arm_2DOF < handle
    
    % properties that, once set in constructor, remain constant
    properties (GetAccess=public, SetAccess=private)
        
        m1;       % upperarm mass [kg]
        m2;       % forearm mass [kg]
        l1;       % upperarm length [m]
        l2;       % forearm length [m]
        s1;       % shoulder to upperarm COM [m]
        s2;       % elbow to forearm COM [m]
        r1;       % upperarm radius of gyration (proximal) [m]
        r2;       % forearm radius of gyration (proximal) [m]
        I1;       % upperarm moment of inertia about shoulder [kg-m^2]
        I2;       % forearm moment of inertia about elbow [kg-m^2]
        B;        % damping matrix [Nms/rad]
        hand;     % handedness [right or left]
        thLim;    % joint angle/velocity limits [rad,rad/s]
        torqLim;  % joint torque limits [Nm]
        
    end
    
    % properties that are initialized in constructor but change over the
    % course of simulation
    properties (GetAccess=public, SetAccess=public)
        
        coupling; % coupling matrix for joint torques
        Td;       % delay between control and sensing [sec]
        q;        % joint angles [rad]
        x;        % state, in joint coordinates [rad,rad/s]
        y;        % sensed output, in joint or task coordinates [rad,rad/s or m,m/s]
        z;        % state, in joint coordinates, augmented for time delay [rad,rad/s]
        shld;     % position of shoulder, in task coordinates [m]
        elbw;     % position of elbow, in task coordinates [m]
        inWS;     % 1 = in workspace, 0 = outside workspace
    
    end
    
    % methods that can be called from outside of the class
    methods (Access=public)
        
        % constructor
        function arm = arm_2DOF(subj)
            if  nargin > 0
                
                % set physical properties
                arm.m1 = 0.028*subj.M; % (Winter, 2009)
                arm.m2 = 0.022*subj.M;
                arm.l1 = 0.188*subj.H;
                arm.l2 = 0.253*subj.H;
                arm.s1 = 0.436*arm.l1;
                arm.s2 = 0.682*arm.l2;
                arm.r1 = 0.542*arm.l1;
                arm.r2 = 0.827*arm.l2;
                arm.I1 = arm.m1*arm.r1^2;
                arm.I2 = arm.m2*arm.r2^2;
                arm.B = [0.05 0.025;0.025 0.05]; % (Crevecoeur, 2013)
                arm.hand = subj.hand;
                arm.thLim = [subj.thMin    subj.thMax;
                             subj.thdotMin subj.thdotMax];
                arm.torqLim = [subj.torqMin subj.torqMax];
                
                % set neurological properties
                if (subj.coupled) arm.coupling = subj.C;
                else              arm.coupling = eye(2); % independent control
                end
                arm.Td = subj.Td;
                
                % initialize state vectors
                
                
            else
                warning('Must specify subject parameters.')
            
            end
        end
 
        % function prototypes
        flag = withinLimits(arm, q)
        [ref, inWS] = defineRef(arm, movt)
        [x, elbw, reachable] = fwdKin(arm, q)
        [q, elbw, reachable] = invKin(arm, x)
        f = dynamics(arm, q, u)
        M = draw(arm)
        
    end
    
    % methods that are only available to other class/subclass methods
    methods (Access=protected)
        
        J = jacobian(arm, q)
        J_dot = jacobianDeriv(arm, q)
        
    end
    
end