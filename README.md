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
