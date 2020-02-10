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
VAL.Lls = 0.0024;

%Exciter Inductance (H)
VAL.Lmfd = 0.6197;
VAL.Lsfd = 0.0378;
VAL.Llfd = 0.0545;

%Other variables
VAL.D = 7;  % Damping Coefficient due to friction (N*m*s/rad)
VAL.J = 0.0658*10^6; % Inertia of the prime mover (Kg*m^2)
VAL.Rl = 40.8; % Resistance of the load (Ohms)
VAL.Rfd = 0.1345;
Val.Rs = 2.43*10^-3;
VAL.N = 16.4 %turn ratio

% initial conditions
Lambda_as = 0;  % Flux Linkage from as
Lambda_bs = 0;  % Flux Linkage from bs              
Lambda_cs = 0;  %Flux Linkage from cs                 
Lambda_fd = 0;  % Flux Linkage from fd              
theta = 0;      %Initial position of the rotor (Angle)
wr = 120*pi;    %Initial velocity of the rotor (V)
z0 = [Lambda_as; Lambda_bs; Lambda_cs; Lambda_fd; theta; wr];


% simulation horizon
T = [0 25]; % (s)

% solve the ODE
options = odeset('RelTol',1e-4);
[t z] = ode15s(@(t,z)G_sys(t,z,VAL),T,z0,options);


%plot the results;

% voltage
subplot(2,1,1)
plot(t,z(:,1))
xlabel('t(s)');
ylabel('Voltage(V)');

% current
subplot(5,1,2)
plot(t,z(:,2))
xlabel('t(s)');
ylabel('Current(A)');

function [dz y] = G_sys(t,z,VAL)
    % Extract state variables
    Lambda_as = z(1);
    Lambda_bs = z(2);
    Lambda_cs = z(3); 
    Lambda_fd = z(4); 
    theta = z(5); 
    wr = z(6);
    
    % Extract other vars. 
    D = VAL.D;
    J = VAL.J;
    Rl = VAL.Rl;
    Rfd = VAL.Rfd;
    Rs = VAL.Rs;
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
    if t > 15
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
    
    %solve for voltage
    Vas = -Rl*Ias - Rs*Ias;
    Vbs = -Rl*Ibs - Rs*Ibs;
    Vcs = -Rl*Ics - Rs*Ics;
    Vfd = -Rl*Ifd - Rfd*Ifd;
    
 
    %final differential equations.
    Te = La*Ifd*((Ias-0.5*Ibs-0.5*Ics)*cos(theta)+(Ias-0.5*Ibs-0.5*Ics)*sin(theta));
    Tf = D*wr;
    Tm = -Te+Tf;
    dwr = 1/J*(Tm+Te-Tf); 
    dtheta = wr;
    
    %Power output of the machine.
    Pe = Vas*Ias+Vbs*Ibs+Vcs*Ics;
    
    dz = [Te; Tf; Tm; dwr; dtheta; Pe];
    
end


