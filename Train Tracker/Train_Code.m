%Armature Current Controlled Motor Transfer Function Approach
clear all
close all
Kma=0.0295 ; % Torque constant (N.m/a)
Kb=0.00295 ; % Back emf constant (Volt.s/rad)
c=0.00006001689451; % Mechanical Damping (kg.m^2/s.rad)
J=0.0001321184025 ; % Rotor Inertia (kg.m^2)
La=0.0047; % Inductance of armature (Henry)
Ra=0.5262773292; % Resistance of armature (Ohm)
K=1;

s=tf('s');
num = K*(Kma/(La*J));          
den = ((s+(Ra/La))*(s+(c/J)))+((Kb*Kma)/La*J);
G = num/den;

%rlocus(G);
margin(G);
