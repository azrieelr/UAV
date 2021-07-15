% Missile Body Parameter
mass    = 340;          % Mass (Kg)
boost   = 30000;        % Booster force (N)
thrust  = 910;          % Thruster force (N)
Lref    = 3.675;        % Length (m)
dref    = 0.28;         % Diameter (m)
Sref    = 0.1711917;    % Cross-section area (m)

% Inertia tensor
Inertia      = zeros(3,3);
Inertia(1,1) = (1/8)*mass*dref^2;
Inertia(2,2) = (1/12)*mass*Lref^2+(1/16)*mass*dref^2;
Inertia(3,3) = (1/12)*mass*Lref^2+(1/16)*mass*dref^2;

% Actuator Parameter
wn_act   = 150;           % wn motor fin, makin besar makin bagus
zeta_act = 0.707;         % zeta motor fin
tau      = 0.04;
fmax     = (10/180)*pi;   % Maximum fin deflection

% Transformation matrix (da,de,dr) to (d1,d2,d3,d4) based on Zipfel (2017)
C3to4  = [1 -1 1;1 1 1;1 1 -1;1 -1 -1];
C4to3  = (1/4)*[1 1 1 1;-1 1 1 -1;1 1 -1 -1];
            
%% Aerodynamic Coefficient of C-705 from CFDv2
load C705CFDv2.mat;

C(49) = -0.2; % CMYq
C(50) = -0.5; % CMZr
C(51) = -0.01; % CMXp