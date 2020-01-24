%{
Author: Brandon Johnson
Date Started: 1/23/2020
Date Completed:

This program Models the dynamic response of a simple electromechanical system. 
This is for HW1 of EE 457.
The goal is to plot traces of x vs. t, vx vs. t, ? vs. t, ef vs. t,
and fe vs. t.
%}

% Initialize system
clear all;
close all;

%Define parameters
val.M = .055;           % Mass of the block (Kg)
val.K = 2667;           % Stiffness of the spring (N/m)
val.D = 4;              % Damping coefficient (Ns/m) 
val.x0= 0.003;          % natural resting position of the block (m)
val.r = 10;             % Resistance of the resistor (Ohms)
val.k = 6.283*10^(-5);  % Constant (H*m)

% initial conditions
x0 = val.x0;                % Position of the block (m)
v0 = 0;                     % Speed of the block (m/s)
lambda0 = 0;                % Initial Flux Linkage (Wb-t)
z0 = [v0;x0;lambda0];       % Initial-condition vector

% exogenous input
Vs = 0;     % Voltage at the source (V)
f = 0;      % Mechanical Input (N);
u = [f;Vs]; % external force vector

% simulation horizon
T = [0 0.5]; % (s)

% solve the ODE
[t z] = ode15s(@(t,z)G_sys(t,z,u,val),T,z0);

% Plot results
% speed
subplot(5,1,1)
plot(t,z(:,1))
xlabel('t(s)');
ylabel('v(m/s)');

% position
subplot(5,1,2)
plot(t,z(:,2))
xlabel('t(s)');
ylabel('x(m)');

%Lambda
subplot(5,1,3)
plot(t,z(:,3))
xlabel('t(s)');
ylabel('Flux Linkage(Wb-t)');

%solve for fe (electrical force);
lambda = z(:,3);
x = z(:,2);
i = (lambda.*x/val.k);     % Create current variable (A)
fe = ((val.k*i.^2)/(2*x.^2));  % Electrical force

%Electrical force
subplot(5,1,4)
plot(t,fe)
xlabel('t(s)');
ylabel('Electrical force(N)');

%Solve for ef. ef = Vs-Vr
Vs = zeros(139,1);  % Create empty array of zeros
Vs(11:139) = 5;     % Make Vs = 5 at t > 2
ef = Vs - val.r.*i;     % solve for ef (Back emf)

%ef
subplot(5,1,5)
plot(t,ef)
xlabel('t(s)');
ylabel('Back emf(V)');

% function of the system dynamic model
function dz = G_sys(t,z,u,val)
    %Assign parameters to variables
    M = val.M;      % Mass of the block (Kg)
    K = val.K;      % Stiffness of the spring (N/m)
    D = val.D;      % Damping coefficient (Ns/m) 
    r = val.r;      % Resistance of the resistor (Ohms)
    k = val.k;      % Constant (H*m)
    x0 = val.x0;    % natural resting position of the block (m)

    % extract external input
    f = u(1);
    Vs = u(2);
    
    %Apply voltage change at time = 0.2(S)
    if t >= 0.2
        Vs = 5;
    end
    
    % extract the states of the system
    v = z(1);       % speed of the block (m/s)
    x = z(2);       % position of the block (m)
    lambda = z(3);    % flux linkage (Wb-t)

    i = (lambda/(k/x)); %create current variable (A)

    % calculate the derivatives of the states
    dv = 1/M*(f- ((k*i^2)/(2*x^2)) - K*(x-x0) - D*v);   % Acceleration (m/s^2)
    dx = v;                                        % Speed (m/s)
    dlambda = Vs-r*i;                              % Flux Linkage (Wb-t) 

    % assemble the derivative vector
    dz = [dv;dx;dlambda]; 
end
