%{
Author: Brandon Johnson
Date Started: 4/20/2020
Date Completed: 4/23/2020

This program Models the dynamics of a synchronous machine
and also the turbine speed control system. 
This is for HW7 of EE 457.
The goal is to incorporate speed control into our our model.
Meaning instead of Assuming Tm = Te - Tf, Tm is now determined
by the steam turbine speed control system.

Note: I decided to start with Hugo Villega's Base code
to insure nothing my previous code messes this up.
%}
%% Initialize system
clear all;
close all;

PAR.Nfs     = 16.4;                  % field-to-stator turn ratio Nfd/Ns
PAR.Lmd     = 3.456e-3;              % (H)
PAR.LA      = 2/3*3.456e-3;          % (H)
PAR.rs      = 2.43e-3;               % stator resistance (Ohm)
PAR.Lls     = 4.079e-4;              % stator leakage inductance (H)
PAR.Llfd    = 3.037e-4*(2/3)*16.4^2; % field-winding leakage inductance (H)
PAR.rfd     = 7.5e-4*(2/3)*16.4^2;   % field-winding resistance (Ohm)
PAR.Lmfd    = 3.456e-3*(2/3)*16.4^2; % (H)
PAR.Lsfd    = 3.456e-3*(2/3)*16.4;   % (H)
PAR.J       = 0.0658e6;              % inertia constant Kgm^2
PAR.Sr      = 835e6;                 % machine rating (VA)
PAR.Vr      = 26e3;                  % machine rated voltage (V)
PAR.P       = 2;                     % machine number of poles
PAR.D       = 7;                     % machine viscous coefficient 

% Speed control constants
R           = 0.05;                  % speed-droop
PAR.K       = 1/R;                   % inverse of speed droop
PAR.T1      = 0.1;                   % speed-control lag time constant (s)
PAR.T2      = 0.03;                  % speed-control lead time constant (s)
PAR.T3      = 0.25;                  % servo-valve time constant (s)
PAR.Tch     = 0.4;                   % steam-chest time constant (s)
PAR.Ptb     = 750e6;                 % turbine rated power (W)

% initial conditions
las0     = 0;
lbs0     = 0;
lcs0     = 0;
lfd0     = 0;
wr0      = 377;
thetar0  = 0;
% Speed control state variables
Dwu0     = 0;
Pv0      = 0;
Pm0      = 0;
z0       = [las0 lbs0 lcs0 lfd0 wr0 thetar0 Dwu0 Pv0 Pm0];

% simulation horizon
T     = [0 50]; % (s)

%% solve the ODE
options = odeset('RelTol',1e-4);
[t z]   = ode15s(@(t,z)M_sys(t,z,PAR),T,z0,options);


%% postprocess data
[~,y0] = M_sys(0,z0,PAR);          % Calculate one output at t=0
y      = zeros(numel(t),numel(y0));  % preassign memory to save outputs
for i=1:numel(t)
    [~, yi] = M_sys(t(i),z(i,:)',PAR);
    y(i,:)  = yi';
end

%% Plot results
fz = 18; % fontsize
figure(1); clf;
Np = 2;
figure(1)
ax1 = subplot(Np,1,1)
plot(t,y(:,4),t,y(:,5),t,y(:,6))
ylabel('v_{abc} (V)')

ax2 = subplot(Np,1,2)
plot(t,y(:,1),t,y(:,2),t,y(:,3))
ylabel('i_{abc} (A)')

figure(2)
ax3 = subplot(Np,1,1)
plot(t,y(:,7))
ylabel('i_{fd} (A)')

ax4 = subplot(Np,1,2)
plot(t,y(:,8))
ylabel('w_r (rad/s)')
ylim([360 380])

figure(3)
ax5 = subplot(Np,1,1)
plot(t,y(:,10))
ylabel('P_e (W)')

ax6 = subplot(Np,1,2)
plot(t,y(:,11))
ylabel('T_e (Nm)')

figure(4)
ax7 = subplot(Np,1,1)
plot(t,y(:,12))
ylabel('Pvu (p.u)')

ax8 = subplot(Np,1,2)
plot(t,(y(:,8)/(120*pi)))
ylabel('wru (p.u)')

figure(5)
ax9 = subplot(1,1,1)
plot(t, y(:,13))
hold on 
plot(t, (-1*y(:,10)/(PAR.Ptb)))
ylabel('Pm,u/ -Pe,u (p.u.)')

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6 ax7 ax8 ax9],'x')


function [dz y] = M_sys(t,z,PAR)
  %% extract system parameters
  Nfs     = PAR.Nfs;  % field-to-stator turn ratio Nfd/Ns
  Lmd     = PAR.Lmd;  % (H)
  LA      = PAR.LA;   % (H)
  rs      = PAR.rs;   % stator resistance (Ohm)
  Lls     = PAR.Lls;  % stator leakage inductance (H)
  Llfd    = PAR.Llfd; % field-winding leakage inductance (H)
  rfd     = PAR.rfd;  % field-winding resistance (Ohm)
  Lmfd    = PAR.Lmfd; % (H)
  Lsfd    = PAR.Lsfd; % (H)
  J       = PAR.J;    % inertia constant Kgm^2
  Sr      = PAR.Sr;   % machine rating (VA)
  Vr      = PAR.Vr;   % machine rated voltage (V)
  P       = PAR.P;    % machine number of poles
  D       = PAR.D;    % machine damping
  K       = PAR.K;    % inverse of speed droop
  T1      = PAR.T1;   % speed-control lag time constant (s)
  T2      = PAR.T2;   % speed-control lead time constant (s)
  T3      = PAR.T3;   % servo-valve time constant (s)
  Tch     = PAR.Tch;  % steam-chest time constant (s)
  Ptb     = PAR.Ptb;  % turbine rated power (W)
  
  %% extract states
  las = z(1); % flux linkage for wdg as
  lbs = z(2); 
  lcs = z(3);
  lfd = z(4); % flux linkage for field wdg
  wr  = z(5); % omegar
  thetar = z(6); 
  Dwu = z(7); % per unit speed control state
  Pv  = z(8); % per unit valve position
  Pm  = z(9); % per unit mechanical power
  
  %% define mutual inductances
  Lasas = Lls + LA;
  Lbsbs = Lls + LA;
  Lcscs = Lls + LA;
  Lfdfd = Llfd + Lmfd;
  Lasbs = -0.5*LA;
  Lbsas = Lasbs;
  Lascs = Lasbs;
  Lcsas = Lascs;
  Lbscs = Lasbs;
  Lcsbs = Lbscs;
  Lasfd = Lsfd*sin(thetar);
  Lfdas = Lasfd;
  Lbsfd = Lsfd*sin(thetar - 2*pi/3);
  Lfdbs = Lbsfd;
  Lcsfd = Lsfd*sin(thetar + 2*pi/3);
  Lfdcs = Lcsfd;
  L = [Lasas Lasbs Lascs Lasfd; 
       Lbsas Lbsbs Lbscs Lbsfd;
       Lcsas Lcsbs Lcscs Lcsfd;
       Lfdas Lfdbs Lfdcs Lfdfd];

  %% obtain current vector
  lambda = [las;lbs;lcs;lfd];
  i = [];
  i = L\lambda;
  ias = i(1);
  ibs = i(2);
  ics = i(3);
  ifd = i(4);


  %% Problem 3
  RL = 40.48;
  if t>= 15 && t<30
      RL = 2.70;
  elseif t>=30 && t<(30+5/60)
      RL = 0;
  elseif t>(30+5/60)
      RL = 2.70;
  end


   %% source voltages
   vas = -RL*i(1);
   vbs = -RL*i(2);
   vcs = -RL*i(3);
   vfd = 230;

   %% Torques
   ifdp =  (2/3)*(PAR.Nfs)*ifd;
   Te   =  (3/2)*PAR.LA*ifdp*((ias-0.5*ibs-0.5*ics)*cos(thetar) + (sqrt(3)/2)*(ibs-ics)*sin(thetar)) ; % electrical torque
   Tf   =  D*wr;
   % calculating Tm
   wru_setting = 1;
   wru  = wr/(120*pi);
   Dwru = wru_setting - wru;
   Pvws = K*Dwu + K*T2/T1*(Dwru-Dwu);
   PvMW = 0; 
   Pvs  = Pvws + PvMW;
   Tm   = Pm*Ptb/wr;
  
   %% Calculate the derivatives of the states
   dlas    = vas - rs*ias;
   dlbs    = vbs - rs*ibs;
   dlcs    = vcs - rs*ics;
   dlfd    = vfd - rfd*ifd;
   dwr     = 1/J*(Tm + Te - Tf);
   dthetar = wr;
   dDwu    = 1/T1*(-Dwu + Dwru);
   dPv     = 1/T3*(-Pv  + Pvs);
   dPm     = 1/Tch*(-Pm + Pv);
   
   %% Electric Power at the terminal of the machine
   Pe   = vas*ias+vbs*ibs+vcs*ics;

   %% assemble the derivative vector
   dz = [dlas; dlbs; dlcs; dlfd; dwr; dthetar; dDwu; dPv; dPm];

   %% assemble set of outputs for postprocessing
   y  = [ias; ibs; ics; vas; vbs; vcs; ifd; wr; thetar; Pe; Te; Pvs; Pm; wru ];

end
