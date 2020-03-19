# Radar-Target-Generation-and-Detetction
Based on Radar System Requirements, a radar target generation and detection system is developed using FMCW waveform configuration, signal propagation technique, Range/Doppler FFT  method and finally 2D CFAR implementation.

## Radar Setup

```
%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Radar Specifications

d_res = 1;                   % Range Resolution
c = 3e8;                     % Speed of light
Max_range = 200;             % Maximum Range of Radar
Max_velocity = 100;          % Maximum Velocity of target vehicle
```

## Target Information

```
% define the target's initial position and velocity. Note : Velocity
% remains contant

R = 140;                     % target's initial position
v = 40;                      % target's velocity
```

## Frequency-Modulated Continuous Wave (FMCW) Generation

```
% Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of 
% the FMCW chirp using the requirements above.

B_sweep = c/2*d_res;         % Sweep Bandwidth
T_chirp = 5.5*2*R/c;         % Chirp Time
slope = B_sweep/T_chirp;     % Slope of FMCW
```
