%{
Author: Brandon Johnson
Date Started: 2/5/2020
Date Completed: 

This program Models the dynamic response of a Synchronous machince. 
This is for HW2 of EE 457.
The goal is to simulate the systems response to steady state, small
disturbances, and a three phase fault.
%}
% Initialize system
clear all;
close all;

%Inductance (H)
VAL.La = 0.0023;
%Leakage Inductance (H)
VAL.Lls = 0.0004079;

%Exciter Inductance (H)
VAL.Lmfd = 0.6197;
VAL.Lsfd = 0.0378;
VAL.Llfd = 0.0545;

%Other variables
VAL.D = 7;              % Damping Coefficient due to friction (N*m*s/rad)
VAL.J = 0.0658*10^6;    % Inertia of the prime mover (Kg*m^2)
VAL.Rl = 40.8;          % Resistance of the load (Ohms)
VAL.Rfd = 0.1345;       % Resistance of the exciter (Ohms)
VAL.Rs = 2.43*10^-3;    % Resistance of the Stator
VAL.N = 16.4;           % Turn ratio

% initial conditions
Lambda_as = 0;          % Flux Linkage from as
Lambda_bs = 0;          % Flux Linkage from bs              
Lambda_cs = 0;          % Flux Linkage from cs                 
Lambda_fd = 0;          % Flux Linkage from fd              
theta = 0;              % Initial position of the rotor (Angle)
wr = 120*pi;            % Initial velocity of the rotor (V)

% Assemble the states vector
z0 = [Lambda_as; Lambda_bs; Lambda_cs; Lambda_fd; wr; theta];


% Simulation horizon
T = [0 25]; % (s)

% solve the ODE
options = odeset('RelTol',1e-4);
[t z] = ode15s(@(t,z)G_sys(t,z,VAL),T,z0,options);


%post-process data
% postprocess data
[~,y0] = G_sys(0,z0,VAL);       % Calculate one output at t=0
y = zeros(numel(t),numel(y0));  % preassign memory to save outputs
for i=1:numel(t)
    [~, yi] = G_sys(t(i),z(i,:)',VAL);
    y(i,:) = yi';
end



%plot the results
num_plots = 6; % The number of plots


% voltage
subplot(num_plots, 1, 1)
plot(t,y(:,4),t,y(:,5),t,y(:,6))
xlabel('t(s)');
ylabel('v_{abc} (V)');
ylim([-30000 30000]);

% current
subplot(num_plots, 1, 2)
plot(t,y(:,1), t,y(:,2),t,y(:,3))
xlabel('t(s)');
ylabel('I_{abc} (A)');

% current
subplot(num_plots, 1, 3)
plot(t,y(:,7))
xlabel('t(s)');
ylabel('Field Winding Current');

% Speed (Rad/s)
subplot(num_plots, 1, 4)
plot(t,z(:,5))
xlabel('t(s)');
ylabel('Rotor Speed');

% Electrical Torque Te
subplot(num_plots, 1, 5)
plot(t,y(:,9))
xlabel('t(s)');
ylabel('Te');


%current
subplot(num_plots, 1, 6)
plot(t,y(:,10))
xlabel('t(s)');
ylabel('Power (Watts)');


function [dz y] = G_sys(t,z,VAL)
    % Extract state variables
    Lambda_as = z(1);
    Lambda_bs = z(2);
    Lambda_cs = z(3); 
    Lambda_fd = z(4); 
    wr = z(5);
    theta = z(6); 

    
    % Extract other vars. 
    D = VAL.D;
    J = VAL.J;
    Rl = VAL.Rl;
    Rfd = VAL.Rfd;
    Rs = VAL.Rs;
    N = VAL.N;
    % Extract Inductance (H)
    La = VAL.La;
    % Extract Leakage Inductance (H)
    Lls = VAL.Lls;
    % Extract Exciter Inductance (H)
    Lmfd = VAL.Lmfd;
    Lsfd = VAL.Lsfd;
    Llfd = VAL.Llfd;
    
    %At T = 15s change the resistance to 2.7 ohms. This simulates a load
    %pick up event.
    if t > 15 && t < 20
        Rl = 2.7;
    end
    
    if t > 20 && t < (20+5/60)
        Rl = 0;
    end
    
    if t > (20+5/60)
        Rl = 2.7;
    end
   
    
    %Self Inductance (H)
    Lasas = La + Lls;
    Lbsbs = La + Lls;
    Lcscs = La + Lls;
    %Mutual Inductance (H)
    Lasbs = -0.5*La;
    Lascs = -0.5*La;
    Lbscs = -0.5*La;
    %Inductance from the stator to the coils
    Lasfd = Lsfd*sin(theta);
    Lbsfd = Lsfd*sin(theta-2*pi/3);
    Lcsfd = Lsfd*sin(theta+2*pi/3);
    Lfdfd = Llfd + Lmfd;

    %Inductance Matrix
    Inductance_Matrix = [Lasas Lasbs Lascs Lasfd; 
                         Lasbs Lbsbs Lbscs Lbsfd;
                         Lascs Lbscs Lcscs Lcsfd;
                         Lasfd Lbsfd Lcsfd Lfdfd;];

                     
    %Form flux linkage matrix.
    Linkage = [Lambda_as; Lambda_bs; Lambda_cs; Lambda_fd];
    
    %solve for the current
    I = Inductance_Matrix\Linkage;
    Ias = I(1);
    Ibs = I(2);
    Ics = I(3);
    Ifd = I(4);
    
    %Iprime
    Ip_fd = 2/3*N*Ifd;
    
    %solve for voltage
    Vas = -Rl*Ias;
    Vbs = -Rl*Ibs;
    Vcs = -Rl*Ics;
    Vfd = 230;
    
    %solve for the change in the flux linkage
    dlambda_as = Vas - Rs*Ias;
    dlambda_bs = Vbs - Rs*Ibs;
    dlambda_cs = Vcs - Rs*Ics;
    dlambda_fd = Vfd - Rfd*Ifd;
    
    %final differential equations.
    %need to change Te
    Te = 3/2*La*Ip_fd*((Ias-0.5*Ibs-0.5*Ics)*cos(theta)+sqrt(3)/2*(Ibs-Ics)*sin(theta));
    Tf = D*wr;
    Tm = -Te+Tf;
    dwr = 1/J*(Tm+Te-Tf); 
    dtheta = wr;
    
    %Power output of the machine.
    Pe = Vas*Ias+Vbs*Ibs+Vcs*Ics;
    
    %return the 4 flux linkages!
    dz = [dlambda_as; dlambda_bs; dlambda_cs; dlambda_fd; dwr; dtheta];
    y = [Ias; Ibs; Ics; Vas; Vbs; Vcs; Ifd; wr; Te; Rl];
end


