% %% MOTOR PARAMETER 1 offset = 270/280
% 
% %misura da LCR meter a 1kHz: Ls = 220.9 microH, Lsmin = 210, Lsmax = 250,
% %Rs = 332.63 microOHM
% % mot.R = 0.25/3;             % (Ohm) Stator resistance
% % mot.Ld = 0.12e-3;          % (H) d-axis inductance 
% % mot.Lq = 0.08e-3;          % (H) q-axis inductance
% mot.R = 0.47;%0.25/3;             % (Ohm) Stator resistance
% mot.Ld = 0.25e-2;%/3; %0.12e-3;          % (H) d-axis inductance 
% mot.Lq = 0.25e-2;%/3; %0.08e-3;          % (H) q-axis inductance
% % mot.Lqd = 0;               % (H) q-axis inductance cross coupling
% mot.lammg = 0.05;        % (Vs) PM flux linkage
% mot.p = 5;                 % Motor pole pairs
% mot.IN = 10;                % (A) motor rated currents
% mot.UN = 24;               % (V) motor rated voltage
% mot.WN = 3e3;              % (rpm) motor rated speed
% mot.tauN = 1.77;           % (Nm) motor rated torque
% mot.kTau = 3/2*mot.p*mot.lammg;% MTPA strategy
% mot.B = 2e-2;         % (N*m*s/rad) Viscous friction
% mot.tauN = mot.B * mot.WN / 0.03;  
% mot.J = 2.18e-4;           % (kgm^2) Motor inertia
% mot.Tcog = 23/(sqrt(2) * 1000); %Nm (rms)
% 
% %% MOTOR PARAMETER 2 (corretti con riferimenti Vd =500 e Vq =1000 ) offset = 270/280
% 
% %misura da LCR meter a 1kHz: Ls = 220.9 microH, Lsmin = 210, Lsmax = 250,
% %Rs = 332.63 microOHM
% mot.R = 0.37;%0.25/3;             % (Ohm) Stator resistance
% mot.Ld = 0.25e-3;%/3; %0.12e-3;          % (H) d-axis inductance 
% mot.Lq = 0.25e-3;%/3; %0.08e-3;          % (H) q-axis inductance
% % mot.Lqd = 0;               % (H) q-axis inductance cross coupling
% mot.lammg = 0.0558;        % (Vs) PM flux linkage
% mot.p = 5;                 % Motor pole pairs
% mot.IN = 10;                % (A) motor rated currents
% mot.UN = 24;               % (V) motor rated voltage
% mot.WN = 3e3;              % (rpm) motor rated speed
% mot.tauN = 1.77;           % (Nm) motor rated torque
% mot.kTau = 3/2*mot.p*mot.lammg;% MTPA strategy
% mot.B = 5e-5;         % (N*m*s/rad) Viscous friction
% mot.tauN = mot.B * mot.WN / 0.03;  
% mot.J = 2.18e-4;           % (kgm^2) Motor inertia
% 
% %% MOTOR PARAMETER 3 (corretti con riferimenti Vd =500 e Vq =1000 ) offset = 135
% 
% %misura da LCR meter a 1kHz: Ls = 220.9 microH, Lsmin = 210, Lsmax = 250,
% %Rs = 332.63 microOHM
% mot.R = 0.65;%0.25/3;             % (Ohm) Stator resistance
% mot.Ld = 0.25e-3;%/3; %0.12e-3;          % (H) d-axis inductance 
% mot.Lq = 0.25e-3;%/3; %0.08e-3;          % (H) q-axis inductance
% % mot.Lqd = 0;               % (H) q-axis inductance cross coupling
% mot.lammg = 0.055;        % (Vs) PM flux linkage
% mot.p = 5;                 % Motor pole pairs
% mot.IN = 12.344;                % (A) motor rated currents
% mot.UN = 24;               % (V) motor rated voltage
% mot.WN = 4e3;              % (rpm) motor rated speed
% mot.tauN = 1.77;           % (Nm) motor rated torque
% mot.kTau = 3/2*mot.p*mot.lammg;% MTPA strategy
% mot.B = 2e-3;         % (N*m*s/rad) Viscous friction
% mot.tauN = mot.B * mot.WN / 0.03;  
% mot.J = 2.18e-4;           % (kgm^2) Motor inertia
% 
% % INITIALIZATION
% mot.lamd0 = mot.lammg;     % (Vs) lambda_d
% mot.lamq0 = 0;             % (Vs) lambda_q
% mot.wm0 = 0;               % (rad/s) initial speed
% mot.thm0 = 0;              % (rad) initial position
% 
% %% MOTOR PARAMETER 3 (corretti con riferimenti Vd =500 e Vq =1000 ) offset = 2.3
% 
% %misura da LCR meter a 1kHz: Ls = 220.9 microH, Lsmin = 210, Lsmax = 250,
% %Rs = 332.63 microOHM
% mot.R = 5;%0.25/3;             % (Ohm) Stator resistance
% mot.Ld = 0.25e-3;%/3; %0.12e-3;          % (H) d-axis inductance 
% mot.Lq = 0.25e-3;%/3; %0.08e-3;          % (H) q-axis inductance
% % mot.Lqd = 0;               % (H) q-axis inductance cross coupling
% mot.lammg = 0.055;        % (Vs) PM flux linkage
% mot.p = 5;                 % Motor pole pairs
% mot.IN = 10;                % (A) motor rated currents
% mot.UN = 24;               % (V) motor rated voltage
% mot.WN = 3e3;              % (rpm) motor rated speed
% mot.tauN = 1.77;           % (Nm) motor rated torque
% mot.kTau = 3/2*mot.p*mot.lammg;% MTPA strategy
% mot.B = 2e-1;         % (N*m*s/rad) Viscous friction
% mot.tauN = mot.B * mot.WN / 0.03;  
% mot.J = 2.18e-4;           % (kgm^2) Motor inertia
% 
% %% MOTOR PARAMETER 3 (corretti con riferimenti Vd =500 e Vq =1000 ) offset = 343.8
% %misura da LCR meter a 1kHz: Ls = 220.9 microH, Lsmin = 210, Lsmax = 250,
% %Rs = 332.63 microOHM
% mot.R = 0.37;%0.25/3;             % (Ohm) Stator resistance
% mot.Ld = 0.25e-3;%/3; %0.12e-3;          % (H) d-axis inductance 
% mot.Lq = 0.25e-3;%/3; %0.08e-3;          % (H) q-axis inductance
% % mot.Lqd = 0;               % (H) q-axis inductance cross coupling
% mot.lammg = 0.055;        % (Vs) PM flux linkage
% mot.p = 5;                 % Motor pole pairs
% mot.IN = 10;                % (A) motor rated currents
% mot.UN = 24;               % (V) motor rated voltage
% mot.WN = 3e3;              % (rpm) motor rated speed
% mot.tauN = 1.77;           % (Nm) motor rated torque
% mot.kTau = 3/2*mot.p*mot.lammg;% MTPA strategy
% mot.B = 5e-5;         % (N*m*s/rad) Viscous friction
% mot.tauN = mot.B * mot.WN / 0.03;  
% mot.J = 2.18e-4;           % (kgm^2) Motor inertia
% 
% 
% %% MOTOR PARAMETER 3 (corretti con riferimenti Vd =500 e Vq =1000 ) offset = 112.6
% %misura da LCR meter a 1kHz: Ls = 220.9 microH, Lsmin = 210, Lsmax = 250,
% %Rs = 332.63 microOHM
% mot.R = 0.25;%0.25/3;             % (Ohm) Stator resistance
% mot.Ld = 0.25e-3;%/3; %0.12e-3;          % (H) d-axis inductance 
% mot.Lq = 0.25e-2;%/3; %0.08e-3;          % (H) q-axis inductance
% % mot.Lqd = 0;               % (H) q-axis inductance cross coupling
% mot.lammg = 0.057;        % (Vs) PM flux linkage
% mot.p = 5;                 % Motor pole pairs
% mot.IN = 10;                % (A) motor rated currents
% mot.UN = 24;               % (V) motor rated voltage
% mot.WN = 4e3;              % (rpm) motor rated speed
% mot.tauN = 1.77;           % (Nm) motor rated torque
% mot.kTau = 3/2*mot.p*mot.lammg;% MTPA strategy
% mot.B = 5.5e-2;         % (N*m*s/rad) Viscous friction
% mot.tauN = mot.B * mot.WN / 0.03;  
% mot.J = 1.5e-5;          % (kgm^2) Motor inertia
% 
% %% MOTOR PARAMETER 3 (corretti con riferimenti Vd =500 e Vq =1000 ) offset = 112.6
% %misura da LCR meter a 1kHz: Ls = 220.9 microH, Lsmin = 210, Lsmax = 250,
% %Rs = 332.63 microOHM
% mot.R = 0.33;%0.25/3;             % (Ohm) Stator resistance
% mot.Ld = 0.4e-2;%/3; %0.12e-3;          % (H) d-axis inductance 
% mot.Lq = 0.12e-2; %1.2e-2;%/3; %0.08e-3;          % (H) q-axis inductance
% % mot.Lqd = 0;               % (H) q-axis inductance cross coupling
% mot.lammg = 0.0488;        % (Vs) PM flux linkage
% mot.p = 5;                 % Motor pole pairs
% mot.IN = 10;                % (A) motor rated currents
% mot.UN = 24;               % (V) motor rated voltage
% mot.WN = 3e3;              % (rpm) motor rated speed
% mot.tauN = 1.77;           % (Nm) motor rated torque
% mot.kTau = 3/2*mot.p*mot.lammg;% MTPA strategy
% mot.B = 1.1e-2;         % (N*m*s/rad) Viscous friction
% mot.tauN = mot.B * mot.WN / 0.03;  
% mot.J = 1.5e-5;          % (kgm^2) Motor inertia
% % INITIALIZATION
% mot.lamd0 = mot.lammg;     % (Vs) lambda_d
% mot.lamq0 = 0;             % (Vs) lambda_q
% mot.wm0 = 0;               % (rad/s) initial speed
% mot.thm0 = 0;              % (rad) initial position
% 
% %% MOTOR PARAMETER 3 (corretti con riferimenti Vd =500 e Vq =1000 ) offset = 112.6
% %misura da LCR meter a 1kHz: Ls = 220.9 microH, Lsmin = 210, Lsmax = 250,
% %Rs = 332.63 microOHM
% mot.R = 0.25;%0.25/3;             % (Ohm) Stator resistance
% mot.Ld = 0.1e-3;%/3; %0.12e-3;          % (H) d-axis inductance 
% mot.Lq = 0.5e-2; %1.2e-2;%/3; %0.08e-3;          % (H) q-axis inductance
% %mot.Lqd =5e-3;               % (H) q-axis inductance cross coupling
% mot.lammg = 0.00977*5;        % (Vs) PM flux linkage
% mot.p = 5;                 % Motor pole pairs
% mot.IN = 10;                % (A) motor rated currents
% mot.UN = 24;               % (V) motor rated voltage
% mot.WN = 3e3;              % (rpm) motor rated speed
% mot.tauN = 1.77;           % (Nm) motor rated torque
% mot.kTau = 3/2*mot.p*mot.lammg;% MTPA strategy
% mot.B = 1e-2;         % (N*m*s/rad) Viscous friction
% mot.tauN = mot.B * mot.WN / 0.03;  
% mot.J = 1.5e-5;          % (kgm^2) Motor inertia
% % INITIALIZATION
% mot.lamd0 = mot.lammg;     % (Vs) lambda_d
% mot.lamq0 = 0;             % (Vs) lambda_q
% mot.wm0 = 0;               % (rad/s) initial speed
% mot.thm0 = 0;              % (rad) initial position
% %% MOTOR PARAMETER 3 calcolo per controllo di velocità
% %misura da LCR meter a 1kHz: Ls = 220.9 microH, Lsmin = 210, Lsmax = 250,
% %Rs = 332.63 microOHM
% mot.R = 0.33;%0.25/3;             % (Ohm) Stator resistance
% mot.Ld = 0.25e-3;%/3; %0.12e-3;          % (H) d-axis inductance 
% mot.Lq = 0.25e-3; %1.2e-2;%/3; %0.08e-3;          % (H) q-axis inductance
% %mot.Lqd =5e-3;               % (H) q-axis inductance cross coupling
% mot.lammg = 0.00977*5;        % (Vs) PM flux linkage
% mot.p = 5;                 % Motor pole pairs
% mot.IN = 10;                % (A) motor rated currents
% mot.UN = 24;               % (V) motor rated voltage
% mot.WN = 3e3;              % (rpm) motor rated speed
% mot.tauN = 1.77;           % (Nm) motor rated torque
% mot.kTau = 3/2*mot.p*mot.lammg;% MTPA strategy
% mot.B = 1e-2;         % (N*m*s/rad) Viscous friction
% mot.tauN = mot.B * mot.WN / 0.03;  
% mot.J = 1.5e-5;          % (kgm^2) Motor inertia
% % INITIALIZATION
% mot.lamd0 = mot.lammg;     % (Vs) lambda_d
% mot.lamq0 = 0;             % (Vs) lambda_q
% mot.wm0 = 0;               % (rad/s) initial speed
% mot.thm0 = 0;              % (rad) initial position

%% MOTOR PARAMETER 3 calcolo per controllo di velocità
%misura da LCR meter a 1kHz: Ls = 220.9 microH, Lsmin = 210, Lsmax = 250,
%Rs = 332.63 microOHM
mot.R = 0.13; %0.33/2;%0.25/3;             % (Ohm) Stator resistance (diviso 2)
mot.Ld = (0.21e-3)/2;%/3; %0.12e-3;          % (H) d-axis inductance 
mot.Lq = (0.21e-3)/2;%(0.21e-3)/2;%(0.21e-3)/2;%(1.27e-3)/2%(0.21e-3)/2;%(1.27e-3)/2;%(1.25e-3)/2; %(0.21e-3)/2; %1.2e-2;%/3; %0.08e-3;          % (H) q-axis inductance
%mot.Lqd =5e-3;               % (H) q-axis inductance cross coupling
mot.lammg = 0.007;%0.007;%69; %0.0066 %0.00977;        % (Vs) PM flux linkage
mot.p = 5;                 % Motor pole pairs
mot.IN = 10;                % (A) motor rated currents
mot.UN = 24;               % (V) motor rated voltage
mot.WN = 3e3;              % (rpm) motor rated speed
mot.kTau = 3/2*mot.p*mot.lammg;% MTPA strategy
mot.tauN = mot.IN*mot.kTau;           % (Nm) motor rated torque
mot.B = 0.001;%0.0001; %0.001;%0.000117;%0.00027;%0.00013;%0.0003;%0.00123;  %0.7745e-3;  0.0025;%      % (N*m*s/rad) Viscous friction
%mot.tauN = mot.B * mot.WN / 0.03;  
mot.J = 1.5/100000; % 1.5/30000; (similar to speed validation) %+ (9.5147e-4)/2;  %(kgm^2) Motor inertia
% INITIALIZATION
mot.lamd0 = mot.lammg;     % (Vs) lambda_d
mot.lamq0 = 0;             % (Vs) lambda_q
mot.wm0 = 0;               % (rad/s) initial speed
mot.thm0 = 0; %5098*2*pi/65536;              % (rad) initial position
mot.Tcog = 23/(sqrt(3)*1000);

% GENERATE LUTs OF id iq
idVec               = [ceil(-mot.IN):1:ceil(mot.IN)];   % (A)
iqVec               = [2*ceil(-mot.IN):1:ceil(mot.IN)*2];   % (A)
lamdVec             = mot.Ld*idVec+mot.lammg;             % to evaluate with OFF-line tests
lamqVec             = mot.Lq*iqVec;                       % to evaluate with OFF-line tests

[idMap, iqMap]  = meshgrid(idVec, iqVec);

% for i=1:length(idVec)
%     iqMap(:,i) = iqMap(:,i)-mot.Lqd/mot.Lq*idVec(i);
% end

figure(8)
surf(lamdVec, lamqVec, iqMap);

%% INVERTER PARAMETER
inv.Ubus = 36;                                   % (V) bus voltage
%inv.Ubus_1 = 1/inv.Ubus;                         % (1/V) inverse of bus voltage
inv.Fpwm = 16000;                                  % (Hz) Switching frequency
inv.Tpwm = 1/inv.Fpwm;                            % (s) Time period of the switching
inv.Fs = inv.Fpwm;                                % (Hz) Sampling frequency
mot.Ts = 1/inv.Fs;   
inv.Ts = 1/inv.Fs;                                % (s) Time period of the sampling
inv.tauD = 3/2*inv.Tpwm;                          % (s) inverter time delay
inv.U_lim = inv.Ubus/sqrt(3);

% LUTs for semplifying Tm, Tm+1 calculation
inv.sector_LUT = zeros(6,4);
for m=1:6
   inv.sector_LUT(m,1) = sin(m*pi/3);
   inv.sector_LUT(m,2) = cos(m*pi/3);
   inv.sector_LUT(m,3) = sin((m-1)*pi/3);
   inv.sector_LUT(m,4) = cos((m-1)*pi/3); 
end

% inverter non idealities
%devices voltage drops
inv.nid.Vsw = 0.08;                               % (V) Switch voltage drop
inv.nid.Vf = 1.5;                                 % (V) Freewheeling diode voltage drop
inv.nid.Vcm = (inv.nid.Vf+inv.nid.Vsw)/2;
inv.nid.Vdm = (inv.nid.Vf-inv.nid.Vsw)/2;            
%dead time and switdches turn on/off
inv.nid.DT = 800e-9;                              % (s) dead time
inv.nid.toff = 0.34e-6;                           % (s) turn off time of the switch
inv.nid.ton = 0.2e-6;                             % (s) turn on time of the switch
inv.nid.ttot = inv.nid.DT + inv.nid.ton - inv.nid.toff;

%Block conversion
Rshunt = 0.01;
Aop = 5.18;
Again = 1; %inv.Ubus/2^16;
Bgain = 1; %(Rshunt * Aop) /3.3;
inv.Ubus = 36.22;
stconv.s16v_2_v = 1/2^15;
stconv.v_2_s16v = 2^15;
stconv.absVdq_max = inv.Ubus/sqrt(3); %circle saturation%
stconv.mA_2_s16A = (65536 * 0.01 *5.18)/(3.3*1000);

%% PI current DESIGN:
% requirement
PI.cur.wgc = 950;%300*(2*pi);                            % (rad/s) Control bandwidth
PI.cur.phim = 70*(pi/180);                          % (rad) Phase margin
PI.cur.Ts = inv.Tpwm;

PI.cur.Ld = mot.Ld;
PI.cur.Lq = mot.Lq;
PI.cur.R = mot.R;
PI.cur.lammg = mot.lammg;

s = tf('s');
Process_d = 1/(s*PI.cur.Ld + PI.cur.R) * 1/(s*inv.tauD + 1);
figure(1)
bode(Process_d);
title('Bode Diagram')
% subplot(2,1,2)
% nyquist(Process_d);
% title('Nyquist diagram')
[PI.cur.kpd,PI.cur.kid] = getPI(Process_d, PI.cur.wgc, PI.cur.phim, PI.cur.Ts);
%my PI
PI.cur.kpd = 64/1024;
PI.cur.kid = 32/16384;
Process_q = 1/(s*PI.cur.Lq + PI.cur.R) * 1/(s*inv.tauD + 1);
figure(2)
subplot(2,1,1)
bode(Process_q);
subplot(2,1,2)
rlocus(Process_q);
title('current control axis q')
[PI.cur.kpq,PI.cur.kiq] = getPI(Process_q, PI.cur.wgc, PI.cur.phim, PI.cur.Ts);
%my PI
PI.cur.kpq = 64/1024;
PI.cur.kiq = 32/16384;

Cq = PI.cur.kpq + (PI.cur.kiq/PI.cur.Ts)/s;
%% 
OL_Process_d = Process_d * Cq *inv.Ubus;
figure(100)
hold on
grid on
bode(Process_q,'b',OL_Process_d,'r');
hold off
legend('Process q axis','Open loop q axis')
title('Bode of quadrature current loop')

CL_Process_d = OL_Process_d/(1 + OL_Process_d*stconv.mA_2_s16A);
figure(3)
grid on
subplot(2,1,1)
grid on
bode(OL_Process_d,'b');
title('Open-loop current axis q')
subplot(2,1,2)
grid on
bode(CL_Process_d,'r');
title('Close-loop current axis q')
%subplot(3,1,2)
%rlocus(OL_Process_d);
OL_Process_q = Process_q * (PI.cur.kpd + PI.cur.kid/PI.cur.Ts/s);

CL_Process_q = OL_Process_q/(1 + OL_Process_q);

%% discretization

%% UTILITIES FOR SIMULATIONS
% convertion from a,c,b axes to alpha,beta,0 axes v1
Tabc_AB0 = 2/3.*[     1       -1/2       -1/2   ; 
                      0     -sqrt(3)/2 sqrt(3)/2  ;
                 1/2 1/2  1/2 ];
Tabc_AB = [Tabc_AB0(1,:); Tabc_AB0(2,:)];
TAB0_abc = Tabc_AB0^-1;

Rz = [0 -1 0 ;
      1  0 0 ;
      0  0 1];
Ry = [-1 0 0 ;
      0  1 0 ;
      0  0 -1];
Tabc_alphabeta = 2/3.*[      1     -1/2       -1/2  ; 
                        0    -sqrt(3)/2  +sqrt(3)/2;
                       1/2     1/2        1/2 ];
                
Talphabeta_abc = Tabc_alphabeta^(-1);                
%Tabc_alphabeta = Tabc_AB0*Rz*Ry
% Tqd_dq = [ 0  -1 ;
%            1   0];
%       
% Tdq_qd = Tdq_qd^-1;
% 
% Tqd_dq = [ 1  0 ;
%            0  1];
%       
% Tdq_qd = Tdq_qd^-1;

%% PI speed DESIGN:
% requirement
PI.vel.wgc = 110; %125; %80 for w and 100 for w_real                            % (rad/s) Control bandwidth
PI.vel.phim = 60*(pi/180); %65*(pi/180);                          % (rad) Phase margin
PI.vel.Ts = 1/1000;                                  % (s) velocity loop sampling period

PI.vel.J = mot.J;
PI.vel.B = mot.B;
Lcc = Cq * Process_q;
Gcc = Lcc/(1+Lcc);

Gcc_1ord = 1/(s/bandwidth(Gcc) + 1);
Process_w_real = Gcc * 1/(s*PI.vel.J + PI.vel.B);
Process_w = Gcc_1ord * 1/(s*PI.vel.J + PI.vel.B);
[PI.vel.kp,PI.vel.ki] = getPI(Process_w,PI.vel.wgc,PI.vel.phim, PI.vel.Ts);
%PI.vel.kp = PI.vel.kp*(2*3.14*1000)/(mot.kTau*60);
%PI.vel.ki = PI.vel.ki*(2*3.14*1000)/(mot.kTau*60);
PI.vel.kp = 7;%*mot.kTau*60/(2*3.14*1000); %10;
PI.vel.ki = 0.5;%*mot.kTau*60/(2*3.14*1000); %0.5/20;%
PI.vel.fuzzy.kp = PI.vel.kp*60000/(2*pi); 
PI.vel.fuzzy.ki = PI.vel.ki*60000/(2*pi);
PI.vel.kt = PI.vel.ki/PI.vel.kp;
pid_vel = PI.vel.kp + (PI.vel.ki/PI.cur.Ts)/s;

%% 
OL_Process_w = Process_w * pid_vel;
CL_Process_w = OL_Process_w/(1 + OL_Process_w);

figure(4)
subplot(2,1,1)
bode(OL_Process_w);
title('Open loop velocity loop')
subplot(2,1,2)
bode(CL_Process_w);
title('Close loop velocity loop ')
%% MEASURAMENT CHAIN:
meas.cur.adc.FS = 20; % (A)
meas.cur.adc.Nb = 12; % ADC number of bit
meas.cur.adc.q = meas.cur.adc.FS/2^meas.cur.adc.Nb; % (A)
meas.enc.Np = 250*4; % Number of pulses
meas.enc.q = 2*pi/meas.enc.Np;

% digital filter design parameter
% GetSpeed1    AVARAGE DERIVATIVE
% GetSpeed2    HPF FIR
meas.enc.nTabs = 16;    % 16 filtered derivative value of the buffer size
meas.enc.Ts = PI.vel.Ts;    % sampling time of the derivative
% GetSpeed3    1ST DER + IIR BUTTERWORTH
meas.enc.wn = 2*pi/(inv.Ts*meas.enc.nTabs)/(2*pi*inv.Fs/2); % cut-off frequency for butterworth filter

% Hall Decoder
meas.enc.Np = 3*2*5; % Number of pulses
meas.hall.q = 2*pi/meas.enc.Np;
meas.hall.Ts = 1/1000;    % sampling time of the derivative



%% HOMEWORK: dq-PLL

% dati
dqpll.est.g = 600;
dqpll.PI.csi = 0.5;
dqpll.PI.wn = 45;
dqpll.lpf.Bc = 30; %100
dqpll.Ts=0.0001;

dqpll.R = mot.R;
dqpll.Ld = mot.Ld;
dqpll.Lq = mot.Lq;
dqpll.lammg = mot.lammg;

% calcolo dei parametri
dqpll.PI.kp=2*dqpll.PI.csi*dqpll.PI.wn;
dqpll.PI.ki=dqpll.PI.wn^2;
dqpll.PI.k1=2*dqpll.PI.csi*dqpll.PI.wn+dqpll.PI.wn;
dqpll.PI.k2=dqpll.PI.wn^2*(1+2*dqpll.PI.csi);
dqpll.PI.k3=dqpll.PI.wn^3;

dqpll.minw = 0.001;    % (rad/s) minima velocità me per l'approssimazione dell'atan
dqpll.Tabc_AB0 = Tabc_AB0;

dqpll.lpf.Bd = exp(-dqpll.Ts*dqpll.lpf.Bc);
dqpll.lpf.alpha = (dqpll.Ts*dqpll.lpf.Bc)/(1+dqpll.Ts*dqpll.lpf.Bc);

% modello con stato aumentato dello stimatore del disturbo
dqpll.est.A = [-dqpll.R -1; 0 0]./dqpll.Ld;
dqpll.est.B = [1; 0]./dqpll.Ld;
dqpll.est.C = [1 0];
sysContinuo = ss(dqpll.est.A,dqpll.est.B,dqpll.est.C,0);

sysDiscreto=c2d(sysContinuo,dqpll.Ts,'zoh');
[dqpll.est.F,dqpll.est.G,dqpll.est.H,dqpll.est.D]=ssdata(sysDiscreto);
dqpll.est.autoval = exp(-dqpll.Ts*dqpll.est.g);
dqpll.est.L = (acker(dqpll.est.F',dqpll.est.H',[dqpll.est.autoval dqpll.est.autoval]))';
dqpll.est.F1 = dqpll.est.F-dqpll.est.L*dqpll.est.H;
dqpll.est.G1 = [dqpll.est.G dqpll.est.L];
dqpll.est.H1 = [0 1];
dqpll.est.D1 = [0 0];

%% MTPA strategy:

mtpa.Ld = mot.Ld;
mtpa.Lq = mot.Lq;
mtpa.lammg = mot.lammg;

%% Fuzzy PI

% Membership bounds on input and output levels
P_LIMIT = 20000;
I_LIMIT = 400000;
D_LIMIT = 5000;
O_LIMIT = 35000;

% Weighting factors for inference
P_RULE_WEIGHT = 10;%PI.vel.kp*2*3.14*1000/(mot.kTau*60);
I_RULE_WEIGHT = 5;%0.47;%PI.vel.ki*20*2*3.14*1000/(mot.kTau*60);
D_RULE_WEIGHT = 0;

rule.P.LIMIT = P_LIMIT;
rule.P.WEIGHT = P_RULE_WEIGHT;
rule.I.LIMIT = I_LIMIT;
rule.I.WEIGHT = I_RULE_WEIGHT;
rule.D.LIMIT = D_LIMIT;
rule.D.WEIGHT = D_RULE_WEIGHT;
rule.O.LIMIT = O_LIMIT;
rule.O.WEIGHT = 1;
 
%% Sensorless 

Obs.csi = 0.5;
Obs.wn = 110;
Obs.lpf.Bc = 100; %100
Obs.Ts = 100e-6;
Obs.g = 1000;

% calcolo dei parametri
Obs.PI.kp=2*Obs.csi*Obs.wn;
Obs.PI.ki=Obs.wn^2;
Obs.PI.k1=2*Obs.csi*Obs.wn+Obs.wn;
Obs.PI.k2=Obs.wn^2*(1+2*Obs.csi);
Obs.PI.k3=Obs.wn^3;

Obs.lpf.alpha = (Obs.Ts*Obs.lpf.Bc)/(1+Obs.Ts*Obs.lpf.Bc);

% modello con stato aumentato dello stimatore del disturbo
Obs.A = [-mot.R -1; 0 0]./mot.Ld;
Obs.B = [1; 0]./mot.Ld;
Obs.C = [1 0];
Ob = obsv(Obs.A,Obs.C);
unob = length(Obs.A)-rank(Ob);
sysContinuo = ss(Obs.A,Obs.B,Obs.C,0);

sysDiscreto=c2d(sysContinuo,dqpll.Ts,'zoh');
[Obs.F,Obs.G,Obs.H,Obs.D]=ssdata(sysDiscreto);
Obs.Ob = obsv(Obs.F,Obs.H);
unobd = length(Obs.F)-rank(Obs.Ob);
Obs.f = 4;
Obs.e1 = 1 - (mot.R * Obs.Ts)/mot.Lq;
Obs.e2 = 1;
Obs.e1obs = Obs.e1/Obs.f;
Obs.e2obs = Obs.e2/Obs.f;
K1 = ( Obs.e1obs + Obs.e2obs - 2 ) / Obs.Ts;
K2 = mot.Lq *( 1 - Obs.e1obs - Obs.e2obs + Obs.e1obs*Obs.e2obs ) / (Obs.Ts^2);
Obs.autoval = exp(-Obs.Ts*Obs.g);
Obs.L = (acker(dqpll.est.F',dqpll.est.H',[0.9 0.9]))';
%Obs.L = [K1 K2]';
Obs.F1 = Obs.F - Obs.L*Obs.H;
Obs.G1 = [Obs.G Obs.L];
Obs.H1 = [0 1];
Obs.D1 = [0 0];

%check if state error asymptotic stable
Obs.eigenvalues = eig(Obs.F1) % we can proceed if |e| < 1

%alpha/beta
Obs.AB.A = [-mot.R/mot.Ld -1; 0 0]./mot.Ld
Obs.Lab = [Obs.L(1) Obs.L(2)]';

c = 5;
Obs.PLL.kp = 2*c;
Obs.PLL.ki = c^2;




