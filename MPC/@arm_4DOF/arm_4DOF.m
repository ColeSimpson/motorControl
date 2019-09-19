% This class captures the attributes, kinematics, and dynamics of a human
% arm modeled as an RR robot restricted to move in the plane of the
% shoulder.
classdef arm_4DOF < handle

    % properties that, once set in constructor, remain constant
    properties (GetAccess=public, SetAccess=private)
        
        DOFs; % cell array of DOF names for plotting
        hand; % handedness [right or left]
        m1;   % upperarm mass [kg]
        m2;   % forearm mass [kg]
        l1;   % upperarm length [m]
        l2;   % forearm length [m]
        s1;   % shoulder to upperarm COM [m]
        s2;   % elbow to forearm COM [m]
        r1;   % upperarm radius of gyration (proximal) [m]
        r2;   % forearm radius of gyration (proximal) [m]
        I1;   % upperarm moment of inertia about shoulder [kg-m^2]
        I2;   % forearm moment of inertia about elbow [kg-m^2]
        tau;  % time constant for low-pass filter between commanded and actual joint torques [sec]
        B;    % damping matrix [Nms/rad]
        
    end
    
    % properties that are initialized in constructor but change over the
    % course of simulation
    properties (GetAccess=public, SetAccess=public)
        
        Ts;        % sampling time [sec]
        Tr;        % "reaction time" for replanning torque trajectory [sec]
        Td;        % delay between control and sensing [sec]
        
        strength;  % coefficient between 0 and 1 dictating arm strength
        coupling;  % coupling matrix for joint torques
        gamma;     % static reflex threshold, for each DOF +/-  [rad]
        mu;        % slope of line defining dynamic reflex threshold, for each DOF +/- [sec]
        k;         % reflex stiffness, for each DOF +/- [Nm/rad]
        b;         % reflex damping, for each DOF +/- [Nms/rad]
        uReflex;   % reflex joint torques [Nm]
        motrNoise; % standard deviation of motor noise [Nm]
        sensNoise; % standard deviation of sensory noise [rad]
        sensBias;  % slope & intercept vectors defining sensory bias [rad]
        
        q;    % joint angles [rad]
        q0;   % joint angle zeros for reflex force calculation [rad]
        u;    % commanded joint torques, separate flexion and extension [Nm]
        x;    % state, in joint coordinates [rad,rad/s,Nm]
        y;    % state, in Cartesian coordinates [m,m/s,N]
        z;    % state, in joint coordinates, augmented for time delay [rad,rad/s,Nm]
        P;    % covariance matrix representing uncertainty in augmented state, z
        shld; % position of shoulder, in Cartesian coordinates [m]
        elbw; % position of elbow, in Cartesian coordinates [m]
        inWS; % 1 = in workspace, 0 = outside workspace
        
    end
    
    % methods that can be called from outside of the class
    methods (Access=public)
        
        % constructor
        function arm = arm_4DOF(subj, movt)
            if  nargin > 0
                
                % Helpful constants
                toRad = pi/180;
                
                % set immutable properties
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
                arm.tau = 0.06;                  % (Crevecoeur, 2013)
                arm.B = 0.25*eye(4) + 0.25*ones(4,4); % (Crevecoeur, 2013)

                % initialize mutable properties to default values
                arm.Ts = 0.001;
                arm.Tr = 0.1;  % (Wagner & Smith, 2008)
                arm.Td = 0.06; % (Crevecoeur, 2013)
                arm.coupling = eye(4);
                arm.motrNoise = 0.0001;
                posNoise = 0.0001;  % noise on position feedback [deg]
                velNoise = 0.00001; % noise on velocity feedback [deg/s]
                arm.sensNoise = [posNoise*ones(4,1); velNoise*ones(4,1)]*toRad;

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Not sure what exactly to do here to translate this to
                % 4DOF.
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % set default deficits
                arm.strength = 1;
                arm.coupling = [-1, 1, 0, 0, 0, 0, 0, 0;
                                 0, 0, -1, 1, 0, 0, 0, 0;
                                 0, 0, 0, 0,-1, 1, 0, 0;
                                 0, 0, 0, 0, 0, 0,-1, 1];
                 spasticData(:,:,2) = [[-5.70    73.37]*toRad;               % (Levin & Feldman, 2003)
                                      [0        0.25];                      % (Levin & Feldman, 2003)
                                      [2.22e-4  1.62e-4]*(1/toRad)*subj.M;  % (McCrea, 2003)
                                      [7.12e-5  2.47e-5]*(1/toRad)*subj.M]; % (McCrea, 2003)
                spasticData(:,:,1) = spasticData(:,:,2);
                spasticData(3,:,1) = spasticData(3,:,1)*10; % (Given, 1995)
                [arm.gamma, arm.mu, arm.k, arm.b] = ...
                    defineSpasticity(0, spasticData); 
                arm.uReflex = [0;0];
                arm.motrNoise = 0.02; % (Izawa, 2008)
                posNoise = 3;         % (Yousif, 2015)
                velNoise = 0.1;
                arm.sensNoise = [posNoise; posNoise; posNoise; posNoise; ...
                    velNoise; velNoise; velNoise; velNoise]*toRad;
                biasData(:,:,1) = [25 -4;40 -2;55 0]*toRad; % (Yousif, 2015)
                biasData(:,:,2) = [25 -4;40 -2;55 0]*toRad; % (Yousif, 2015)
                biasData(:,:,3) = [25 -4;40 -2;55 0]*toRad; % (Yousif, 2015)
                biasData(:,:,4) = [80 5;95 7;100 8]*toRad;  % (Yousif, 2015)
                arm.sensBias = defineBiasFunc(biasData);

                
                % set default joint limits
                th1Min = -30*toRad;     % Min. shoulder extension [rad]
                th1Max = 100*toRad;     % Max. shoulder extension [rad]
                th2Min = -70*toRad;     % Max. shoulder adduction [rad]
                th2Max = 120*toRad;     % Max. shoulder abduction [rad]
                th3Min = -49*toRad;     % Min. shoulder rotation [rad]
                th3Max = 90*toRad;      % Max. shoulder rotation [rad]
                th4Min = 0*toRad;       % Max elbow flexion [rad]
                th4Max = 170*toRad;     % Max elbow extension [rad]
                arm.q.min = [ th1Min; th2Min; th3Min; th4Min ];
                arm.q.max = [ th1Max; th2Max; th3Max; th4Max ];
                arm.x.min = [ arm.q.min; -inf*ones(12,1) ]; % no explicit velocity limits
                arm.x.max = [ arm.q.max; inf*ones(12,1) ]; 
                
                % set default actuator limits
                torq1Max = 85;     % shoulder flexion torque limits [Nm]
                torq2Max = 100;     % shoulder extension torque limits [Nm]
                torq3Max = 75;     % shoulder adduction torque limits [Nm]
                torq4Max =  75;     % shoulder abduction torque limits [Nm]               
                torq5Max = 60;     
                torq6Max =  50;     % shoulder rotation torque limits [Nm]
                torq7Max = 60;     
                torq8Max =  75;     % elbow torque limits [Nm]
                arm.u.max = [ torq1Max; torq2Max; torq3Max; torq4Max; torq5Max; torq6Max; torq7Max; torq8Max];
                arm.u.min = 0.*arm.u.max;
                
                % initialize state vectors for arm in middle of defined
                % workspace
                arm.shld = [0;0;0];
                arm.q.val = mean([arm.q.min, arm.q.max], 2);
                arm.q0 = arm.q.val;
                arm.u.val = zeros(size(arm.u.max));
                arm.x.val = [arm.q.val; zeros(4,1); zeros(size(arm.u.val))];
                [arm.y.val, arm.elbw, arm.inWS] = arm.fwdKin;
                nDelay = ceil(arm.Td/arm.Ts);
                arm.z.val = repmat(arm.x.val, nDelay+1, 1); % (nDelay + 1) includes current state
                arm.P = zeros(length(arm.z.val));           % 0 = no uncertainty in state information
                
            else
                warning('Must specify subject parameters.')                
            end
        end
        
        % function prototypes
        flag = withinLimits(arm, q)
        [x, elbow, reachable] = fwdKin(arm, q)
        [q, elbow, reachable] = invKin(arm, x)
        f = dynamics(arm, u, ctrlSpace)
        M = draw(arm, x)
        
    end
    
    % methods that are only available to other class/subclass methods
    methods (Access=protected)
        
        J = jacobian(arm, q)
        J_dot = jacobianDeriv(arm, q)
        
    end
    
end