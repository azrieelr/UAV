% MATLAB 2016A, Windows 8.1
% by: Mochamad Nur Qomarudin, mochamadnurq@gmail.com

clc
clear

%% INPUT SIMULASI RUDAL
h0     = 1;    % ketinggian awal (m)
phi0   = 5;    % sudut roll awal (deg), +-5 deg, dengan asumsi bahwa sudut roll
               % bisa dipastikan ~0 deg sebelum peluncuran
theta0 = 10;   % sudut elevasi awal (deg), dari tanah, minimal 10 deg 
psi0   = 0;    % sudut heading awal (deg), perlu diatur sesai arah target, clockwise dari utara

% WAYPOINTS dalam Frame Flat Earth: [Z,Y,Z]
% X+: Utara, Y+: Timur
p0 = [0,0,-h0]           % Initial position in flat Earth frame
p1 = [6000,100,-200]     % Waypoint 1
p2 = [12000,300,-200]    % Waypoint 2
p3 = [18000,700,-200]    % Waypoint 3
pt = [24000,1000,0]      % Waypoint 4 (Posisi target)

%% Converted Waypoints in flat Earth Frame

px = [p0(1),p1(1),p2(1),p3(1),pt(1)];
py = [p0(2),p1(2),p2(2),p3(2),pt(2)];
pz = [p0(3),p1(3),p2(3),p3(3),pt(3)];

Euler0   = [deg2rad(phi0) deg2rad(theta0) deg2rad(psi0)];    % rad
vb0    = [1e-3 0 0];
wb0    = [0 0 0];

%% Parameter Aerodinamik
C705_CFD_parameter

%% Parameter motor sirip
wn_act   = 150;          % wn motor fin, makin besar makin bagus
zeta_act = 0.707;        % zeta motor fin
fmax     = deg2rad(10);  % defleksi sirip maksimum
dela_max = deg2rad(10);
dele_max = deg2rad(10);
delr_max = deg2rad(10);
C3to4    = [1 -1 1;1 1 1;-1 1 1;-1 -1 1];
C4to3    = (1/4)*[1 1 -1 -1;-1 1 1 -1;1 1 1 1];

%% PARAMETER WAKTU SAMPLING
Fs_guid  = 200;       % Fs guidance
Fs_ap    = 200;       % Fs autopilot
Fs_imu   = 1000;      % Fs IMU
Ts_guid  = 1/Fs_guid;
Ts_ap    = 1/Fs_ap;
Ts_imu   = 1/Fs_imu;

%% PARAMETER IMU
acc_bias       = [-0.4596 -0.2992 0.0644];                   % bias accelero sumbu x y z
gyr_bias       = [-12.1506e-06 -1.5819e-04 -5.9725e-05];     % bias gyro sumbu x y z
zeta_lpf       = 0.707;
wn_lpf         = 1646.1946; % (rad/sec) ~ 262 Hz
imu_noise_seed = [4010 5010 6010 7010 8010 9010];
imu_noise_pwr  = [0.0000005 0.0000018 0.00000059 0.00000000083638 0.0000000005019 0.000000000068075];   % bila mean nol, power=varian

%% Parameter Seeker Kamera
f_camera = 0.1;

%% Parameter Pemandu Percepatan
N        = 3;

%% Parameter Target Virtual Bergerak
TH       = 3;   % Time Horizon: Jarak rudal dengan target virtual (detik)
k0       = 1;   % Posisi awal target virtual (Waypoint ke-1)

%% Waktu aktivasi kendali sirip (detik)
t_sim   = 100;              % waktu simulasi
tswitch = 0;                % lama waktu alignment (detik)
tboost  = 3;                % lama waktu fase booster (detik)
t_activate_del_a = 3;       % waktu aktivasi sirip untuk roll
t_activate_del_e = 5;       % waktu aktivasi sirip untuk pitch
t_activate_del_r = 5;       % waktu aktivasi sirip untuk yaw

%% Parameter transisi autopilot
d_transisi = 2e3;     % Jarak rudal dengan target saat transisi 
% d_transisi = 0;
% d_transisi = 4e5;

%% Parameter Autopilot Roll
Kiphi =  2e-7;
Kphi  =  1.8594;
Kp    = -1.3248;

%% Parameter Autopilot Vertical Acceleration
Kp_az = 0.0119;
Ki_az = 0.0085;
Kq    = 3.8200;

%% Parameter Autopilot Lateral Acceleration
Kp_ay = -0.0135;
Ki_ay = -0.0024;
Kr    = -10.1447;

%% Parameter Autopilot Yaw Angle
Kp_psi = 1.7937;
Ki_psi = 4.1865e-07;
Kr_psi = 8.6031;

%% Parameter Autopilot Pitch Angle
Kp_theta =  1.7024;
Ki_theta =  2.1878e-7;
Kq_theta = -3.1594;

%% Parameter Autopilot Height
Kp_h    =  0.45;
Khdot   = -1;

sim('C705_FLATEARTH_sim.slx')