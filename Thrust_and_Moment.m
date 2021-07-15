function [F, M, m, I_dot, I] = Thrust_and_Moment(s1, s2, u1, u2, u3, u4, de, dr, da, dt, g, Tb, Rb, Euler, wind)

F = zeros(3,1);
M = zeros(3,1);
I_dot = zeros(3);
I = zeros(3);
u=Tb(1);
v=Tb(2);
w=Tb(3);
p=Rb(1);
q=Rb(2);
r=Rb(3);
roll=Euler(1);
pitch=Euler(2);
yaw=Euler(3);
w_ns=wind(1); % steady wind - North
w_es=wind(2); % steady wind - East
w_ds= wind(3); % steady wind - Down
u_wg=wind(1); % gust along body x-axis
v_wg=wind(2); % gust along body y-axis    
w_wg=wind(3); % gust along body z-axis

% compute wind data in NED
w_n =cos(pitch)*cos(yaw)*w_ns+cos(pitch)*sin(yaw)*w_es-sin(pitch)*w_ds+u_wg;
w_e =(sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw))*w_ns+...
     (sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw))*w_es+...
     sin(roll)*cos(pitch)*w_ds+v_wg;
w_d =(cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw))*w_ns+...
     (cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw))*w_es+...
     cos(roll)*cos(pitch)*w_ds+w_wg; 
 
% compute air data
ur=u-w_n; vr=v-w_e; wr=w-w_d;
Va=sqrt(ur^2+vr^2+wr^2);
alpha=atan2(wr,ur);
B=asin(vr/Va);

% Every Parameter & Coeficient
m=1.56; %massa
g=9.81; %grafitasi
Ix=0.1147; %inertia sb x
Iy=0.0576; %inertia sb y
Iz=0.1712; %inertia sb z
Ixz=0.0015; %inertia sb xz
S=0.2589;
b=1.4224;
c=0.3302;
Sprop=0.0314;
Cprop=1;
Kmotor=20;
R=1.2682;
KTp=0;
KOmega=0;

%Translational Coeficient
alpha = 17; %maksimal lift in 15-20
%B= 15;  
CLo=0.09167;
CLa=3.5026;
CLA=CLo+CLa*alpha;
CLq=2.8932;
CLde=0.2724;
CDo=0.01613;
CDa=0.2108;
CDA=CDo+CDa*alpha;
CDq=0;
CDde=0.3045;
Cyo=0;
Cyb=-0.07359;
Cyp=0;
Cyr=0;
Cyda=0;
Cydr=-0.17;

%x-axis
Cxa=-CDA*cos(alpha)+CLA*sin(alpha);
Cxqa=-CDq*cos(alpha)+CLq*sin(alpha);
Cxdea=-CDde*cos(alpha)+CLde*sin(alpha);

%z-axis
Cza=-CDA*sin(alpha)-CLA*cos(alpha);
Czqa=-CDq*sin(alpha)-CLq*cos(alpha);
Czdea=-CDde*sin(alpha)-CLde*cos(alpha);

%Rotational Coeficient
Cmo=-0.02338;
Cma=-0.5675;
Cmq=-1.3990;
Cmde=-0.3254;
Clo=0;
Clb=-0.02854;
Clp=-0.3209;
Clr=0.03066;
Clda=0.1682;
Cldr=0.105; 
Cno=0;
Cnb=-0.0040;
Cnp=-0.01297;
Cnr=-0.00434;
Cnda=-0.00328;
Cndr=-0.032; 

%Inertial Product
A=Ix*Iz-Ixz^2;
A1=(Ixz*(Ix-Iy+Iz))/A;
A2=(Iz*(Iz-Iy)+Ixz^2)/A;
A3=Iz/A;
A4=Ixz/A;
A5=(Iz-Ix)/A;
A6=Ixz/A;
A7=((Ix-Iy)*Ix+Ixz^2)/A;
A8=Ix/A;

%roll 
Cpo=A3*Clo+A4*Cno;
Cpb=A3*Clb+A4*Cnb;
Cpp=A3*Clp+A4*Cnp;
Cpr=A3*Clr+A4*Cnr;
Cpda=A3*Clda+A4*Cnda;
Cpdr=A3*Cldr+A4*Cndr;

%yaw 
Cro=A4*Clo+A8*Cno;
Crb=A4*Clb+A8*Cnb;
Crp=A4*Clp+A8*Cnp;
Crr=A4*Clr+A8*Cnr;
Crda=A4*Clda+A8*Cnda;
Crdr=A4*Cldr+A8*Cndr;

%Gaya drag
Kftx=0.0005567;
Kfty=0.0005567;
Kftz=0.0006354;
Kfax=0.0005567;
Kfay=0.0005567;
Kfaz=0.0006354;

%Translational
F(1,1) = m*(-g*sin(pitch)+s1*(R/2/m)*Va^2*S*(Cxa+Cxqa*((c*q)/(2*Va))+Cxdea*de)+s1*(R/2/m)*Sprop*Cprop*((Kmotor*dt)^2-Va^2) ...
        +s2*((1/m)*(-Kftx)*u));
F(2,1) = m*(g*cos(pitch)*sin(roll)+s1*(R/2/m)*Va^2*S*(Cyo+Cyb*B+Cyp*(b*p/2*Va)+Cyr*(b*r/2*Va)+Cyda*da+Cydr*dr) ...
         +s2*((1/m)*(-Kfty)*v));
F(3,1) = m*(g*cos(pitch)*cos(yaw)+s1*(R/2/m)*Va^2*S*(Cza+Czqa*((c*q)/(2*Va))+Czdea*de) ...
         +s2*((1/m)*(Kftz*w-u1)));

%Rotational

pdot = (A1*p*q)-(A2*q*r)+s1*(R/2)*(Va^2*S*b)*(Cpo+Cpb*B+Cpp*(b*p/2*Va)+Cpr*(b*r/2*Va)+Cpda*da+Cpdr*dr) ...
    +s2*(A3*(u2-Kfax*p^2)+A4*(u4-Kfaz*r^2));
qdot = (A5*p*r)-(A6*(p^2-r^2))+s1*(R/(2*Iy))*(Va^2*S*c)*(Cmo+Cma*alpha+Cmq*(c*p/2*Va)+Cmde*de) ...
    +s2*(1/Iy)*(u3-Kfay*q^2);
rdot = (A7*p*q)-(A1*q*r)+s1*(R/2)*(Va^2*S*b)*(Cro+Crb*B+Crp*(b*p/2*Va)+Crr*(b*r/2*Va)+Crda*da+Crdr*dr) ...
    +s2*(A4*(u2-Kfax*p^2)+A8*(u4-Kfaz*r^2));

%Rotation Matrix
C = [cos(pitch)*cos(yaw) sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw) cos(roll)*sin(pitch)*sin(yaw)+sin(roll)*sin(yaw);
    cos(pitch)*sin(yaw) sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw) cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);
    sin(pitch) -sin(roll)*cos(pitch) -cos(roll)*cos(pitch)];

rolldot=p+q*sin(roll)*tan(pitch)+r*cos(roll)*tan(pitch);
pitchdot=q*cos(roll)-r*sin(roll);
yawdot=q*sin(roll)/cos(pitch)+r*cos(roll)/cos(pitch);
Medot = C*[u v w]';