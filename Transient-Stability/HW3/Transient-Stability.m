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

part = 0;

%Table of stability:
stability = zeros(6,9);

% Parameters
PAR.H   = 5;                                % Inertia constant (s)  
PAR.wb  = 120*pi;                           % Inertia constant (s) 
PAR.Tc  = 15/60;                            % Time elapsed to clear the fault (s)
PAR.Xt  = 0.1;                              % Transformer Reactance
PAR.Xl  = 0.4;                              % Transformer line reactance
PAR.Xd  = 0.2;                              % Machine transient reactance
PAR.Vt  = 1.01;                             % Voltage at the terminals of the machine
pe = [0.5,0.6,0.7,0.8,0.9,1.0];             % Initial per-unit power flowing into the system
epsilon = pi/4;                             % Radius that determines stability

d = [0.1,0.2,0.3,0.5,0.6,0.7,0.8,0.9, 1.0]; % Percent distance where fault occurs
%initial conditions
wr0     = 1.0;                              % initial rotor speed p.u.
T       = 0:0.001:1+0.15;                   % simulation horizon
distance = 0;
for i = 1:1:6
  for j = 1:1:9
    E0      = initial_conditions(pe(i),PAR);
    PAR.E   = E0(1);
    del0    = E0(2);                        % Initial relative rotor angle (rad)
    z0      = [wr0;del0];                   % initial condition vector
    PAR.Zf  = impedance(d(j),PAR);
    % execute ODE
    [t z] = ode15s(@(t,z)rotor_angle_dyn(t,z,PAR),T,z0);
    
    %TODO, 
    %determine if phase distance exceeds epsilon

    %if the phase distance is less than epsilon, then the system is stable
    if epsilon > distance
      stability(i,j) = 1;
    end 
  end
end 

if part > 0
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
end

% System dynamics
function dx = rotor_angle_dyn(t,x,PAR)

  wr  = x(1);  % speed of the machine
  del = x(2);  % relative rotor angle

  % electrical torque calculations
  Pe = 0;
  if t<0.15  % before the fault
    Pe = (PAR.E/0.5)*sin(del); 
  end

  if t>=0.15 && t<0.15+PAR.Tc % during the fault
    Pe = (PAR.E/PAR.Zf)*sin(del);
    %Pe = 0;
  end

  if t>=0.15+PAR.Tc % after the fault
    Pe = (PAR.E/0.7)*sin(del);
  end

  % calculate derivatives
  dwr   = 1/(2*PAR.H)*(0.8 - Pe);
  ddel  = PAR.wb*(wr - 1);

  % vector with state derivatives
  dx = [dwr;ddel];
end

%This question is used to find the initial conditions.
function I0 = initial_conditions(pe, PAR)
  %calculate theta. 
  theta = asin(((PAR.Xd+PAR.Xt)*pe)/PAR.Vt);
  %calculate cart coords    
  [x,y] = pol2cart(theta, PAR.Vt);
  %calc Ias
  Ias = ((x+i*y)-1)/((PAR.Xd+PAR.Xt)*i);
  %calc E
  E = Ias*(PAR.Xd*i) + (x + i*y);
  %calcs internal voltage and angle of machine
  E0 = sqrt(real(E)^2+imag(E)^2);
  del = atan2(imag(E), real(E));
  I0 = [E0;del];
end

%this function calculates the impedance of the line once a fault occurs.
function z = impedance(d,PAR)
  zf = PAR.Xl*d;
  zl = PAR.Xl*(1-d);
  zl_eq = 1/((1/PAR.Xl)+(1/zl));

  z = ((PAR.Xd+PAR.Xt)*zl_eq + (PAR.Xd+PAR.Xt)*zf + zl_eq*zf)/zf;          
end 
