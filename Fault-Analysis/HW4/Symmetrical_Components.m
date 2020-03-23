%{
Author: Brandon Johnson
Date Started: 3/15/2020
Date Completed: 3/23/2020

This is for EE 457 . The goal is to model the 
waveforms as seen in the NERC report and analyze 
them using the method of symmetrical components
%}

%alpha constant
a = -0.500 + i*0.866;
%symmetrical components matrix
A = [1  1   1  ;
     1  a^2 a  ;
     1  a   a^2;];
   
%ask user for the 3 phases.  
A_radius = input('Enter Phase As radius: ');
A_angle  = input('Enter Phase As angle: ');
B_radius = input('Enter Phase Bs radius: ');
B_angle  = input('Enter Phase Bs angle: ');
C_radius = input('Enter Phase Cs radius: ');
C_angle  = input('Enter Phase Cs angle: ');

%convert the phasors into cartesian coords
%so the matrix math becomes easier
a_phasor = tocart(A_radius, A_angle);
b_phasor = tocart(B_radius, B_angle);
c_phasor = tocart(C_radius, C_angle);

%create unbalanced matrix 
v_in = [a_phasor; b_phasor; c_phasor];

%perfrom symmetrical components transformation
v_sequence = A\v_in;

%extract sequence components.
v_zero  = round(v_sequence(1),2);
v_plus  = round(v_sequence(2),2);
v_minus = round(v_sequence(3),2);


%display the + - 0 components in polar form
display(topolar(v_plus))
display(topolar(v_minus))
display(topolar(v_zero)) 

%Plot the positive negative and zero sequence components.
t = [-2.5:.1:38]/1000;

%Plotting the positive sequence components
z = topolar(v_plus);
va_plus_rad = z(1);
va_plus_angle = z(2);
%phase b
vb_plus_rad = va_plus_rad;
vb_plus_angle = rotate(va_plus_angle, -120);
%phase c 
vc_plus_rad = va_plus_rad;
vc_plus_angle = rotate(va_plus_angle, 120);

va_plus_time = sqrt(2) * va_plus_rad * cos(120*pi*t + va_plus_angle*pi/180);
vb_plus_time = sqrt(2) * vb_plus_rad * cos(120*pi*t + vb_plus_angle*pi/180);
vc_plus_time = sqrt(2) * vc_plus_rad * cos(120*pi*t + vc_plus_angle*pi/180);

%plotting positive sequence components
plot(t,va_plus_time,'b',t,vb_plus_time,'r',t,vc_plus_time,'g');
title('Positive Sequence Components')


%Plotting the negative sequence components
z = topolar(v_minus);
va_minus_rad = z(1);
va_minus_angle = z(2);
%phase b
vb_minus_rad = va_minus_rad;
vb_minus_angle = rotate(va_minus_angle, 120);
%phase c 
vc_minus_rad = va_minus_rad;
vc_minus_angle = rotate(va_minus_angle, -120);

va_minus_time = sqrt(2) * va_minus_rad * cos(120*pi*t + va_minus_angle*pi/180);
vb_minus_time = sqrt(2) * vb_minus_rad * cos(120*pi*t + vb_minus_angle*pi/180);
vc_minus_time = sqrt(2) * vc_minus_rad * cos(120*pi*t + vc_minus_angle*pi/180);

%plotting negative sequence components
plot(t,va_minus_time,'b',t,vb_minus_time,'r',t,vc_minus_time,'g');
title('Negative Sequence Components')

%Plotting the zero sequence components
z = topolar(v_zero);
va_zero_rad = z(1);
va_zero_angle = z(2);
%phase b
vb_zero_rad = va_zero_rad;
vb_zero_angle = va_zero_angle;
%phase c 
vc_zero_rad = va_zero_rad;
vc_zero_angle = va_zero_angle;

va_zero_time = sqrt(2) * va_zero_rad * cos(120*pi*t + va_zero_angle*pi/180);
vb_zero_time = sqrt(2) * vb_zero_rad * cos(120*pi*t + vb_zero_angle*pi/180);
vc_zero_time = sqrt(2) * vc_zero_rad * cos(120*pi*t + vc_zero_angle*pi/180);

%plotting positive sequence components
plot(t,va_zero_time,'b',t,vb_zero_time,'r',t,vc_zero_time,'g');
title('zero Sequence Components')

%finally, plot the addition of each component

v_pos = va_plus_time + va_minus_time + va_zero_time;
v_neg = vb_plus_time + vb_minus_time + vb_zero_time;
v_o   = vc_plus_time + vc_minus_time + vc_zero_time;
plot(t,v_pos,'b',t,v_neg,'r',t,v_o,'g');
title('Symmetrical Components')

%converts complex number to polar
function z = topolar(x);
  radius = sqrt(real(x)^2 + imag(x)^2);
  angle = atan2(imag(x),real(x)) * 180/pi;
  z = [radius; angle];
end

%convert polar to cartesian
function z = tocart(r, angle)
  real = round(r*cos(angle*pi/180),2);
  imag = round(r*sin(angle*pi/180),2);
  z = real + i*imag;
end

%this function returns the rotated phasor.
function new_angle = rotate(angle, rotation)
  new_angle = angle + rotation;
end

