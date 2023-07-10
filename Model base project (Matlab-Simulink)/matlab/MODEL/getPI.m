function [kp,ki] = getPI(Process, wgc, phim, Ts)

[mod_P, phase_P] = bode(Process,wgc);
phase_P = phase_P*pi/180;

phase_C = phim - phase_P - pi;
mod_C = 1/mod_P;

switch nargin
    case 3       
        kp = mod_C*cos(phase_C);
        ki = -wgc*mod_C*sin(phase_C);
    case 4
        kp = mod_C*cos(phase_C);
        ki = -wgc*mod_C*sin(phase_C)*Ts;
end