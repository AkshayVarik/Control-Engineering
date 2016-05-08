%Field Current Controlled Motor Transfer Function Approach
Kmf=0.3233728703 ; % Torque constant (N.m/a)
%Kb=0.4952900056 ; % Back emf constant (Volt.s/rad)
c=0.0006001689451; % Mechanical Damping (kg.m^2/s.rad)
J=0.001321184025 ; % Rotor Inertia (kg.m^2)
Lf= 0.0047; % Inductance of armature (Henry)
Rf= 5.262773292; % Resistance of armature (Ohm)

s=tf('s');
num = (Kmf/(Lf*J));          
den = (s*(s+(c/J))*(s+(Rf/Lf)));
G = num/den;

%Time Delay
T=0.01;
N=1;
[num,den]=pade(T,N);

rlocus();
%bode(G);
