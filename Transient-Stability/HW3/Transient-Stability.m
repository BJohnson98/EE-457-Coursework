%{
Author: Brandon Johnson
Date Started: 2/25/2020
Date Completed: 
This program models the relative rotor angle stability between synchronous
machines.
This is for HW3 of EE 457.
The goal is to determine if the relative rotor angle of the synchronous
machine stays stable after a disturbance ot the system. Stability is
determined in the sense of lyapunov. We will test using different 
initial coniditions. We will modify the power injected into the system
and the distance of the fault.
%}

% Prepare system
clear all;
close all;

% Parameters
epsilon = pi/4;             % radius that determines stability
del0   = 21.09*pi/180;      % initial relative rotor angle (rad)
wr0    = 1.0;               % initial rotor speed p.u.
z0     = [wr0;del0];        % initial condition vector
PAR.H  = 5;                 % inertia constant (s)  
PAR.wb = 120*pi;            % inertia constant (s) 
PAR.Tc = 15/60;             % time elapsed to clear the fault (s) 
T      = 0:0.001:1+0.15;    % simulation horizon

% execute ODE
[t z] = ode15s(@(t,z)rotor_angle_dyn(t,z,PAR),T,z0);

% plot figures
figure(1)
subplot(2,2,1); plot(t,z(:,1));
xlabel('t (s)'); ylabel('wr (p.u.)');
subplot(2,2,3); plot(t,z(:,2));
xlabel('t (s)'); ylabel('del (rad)');

subplot(2,2,[2 4]); 
%plot(z(:,1),z(:,2)); 
hold on

th = 0:pi/50:2*pi;
plot(epsilon * cos(th) + wr0, epsilon * sin(th) + del0);
plot(z(:,1),z(:,2)); 
ylabel('del (rad)'); xlabel('wr (p.u.)');
hold off

% System dynamics
function dx = rotor_angle_dyn(t,x,PAR)

    wr  = x(1);  % speed of the machine
    del = x(2);  % relative rotor angle

    % electrical torque calculations
    Pe = 0;
    if t<0.15  % before the fault
       Pe = (1.111/0.5)*sin(del); 
    end

    if t>=0.15 && t<0.15+PAR.Tc % during the fault
       Pe = (1.111/1.1)*sin(del);
       %Pe = 0;
    end

    if t>=0.15+PAR.Tc % after the fault
       Pe = (1.111/0.7)*sin(del);
    end

    % calculate derivatives
    dwr   = 1/(2*PAR.H)*(0.8 - Pe);
    ddel  = PAR.wb*(wr - 1);

    % vector with state derivatives
    dx = [dwr;ddel];
end
