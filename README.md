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

### Simulation Loop
Simulate the target movement and calculate the beat or mixed signal for every instance

```
% Operating carrier frequency of Radar 
fc = 77e9;                   %carrier freq
                                                          
% The number of chirps in one sequence. Its ideal to have 2^ value for 
% the ease of running the FFTfor Doppler Estimation. 
Nd = 128;                    % No of doppler cells OR No of sent periods 
                             % number of chirps

%The number of samples on each chirp. 
Nr = 1024;                    % for length of time OR no of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp

t = linspace(0,Nd*T_chirp,Nr*Nd);        %total time for samples

%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx = zeros(1,length(t));      % transmitted signal
Rx = zeros(1,length(t));      % received signal
Mix = zeros(1,length(t));     % beat signal

%Similar vectors for range_covered and time delay.
r_t = zeros(1,length(t));
td = zeros(1,length(t));

% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    %TODO*:
    %For each time stamp update the Range of the Target for constant 
    %velocity. 
    r_t(i) = R + t(i)*v;
    td(i) = r_t(i)*2/c;
    
    %TODO*:
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos(2*pi*(fc*t(i)+slope*t(i)^2*0.5));
    Rx(i) = cos(2*pi*(fc*(t(i)-td(i))+slope*(t(i)-td(i))^2*0.5));
    
    %TODO*:
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i).*Rx(i);
    
end
```

## Range Measurement - 1st FFT Operation
*Implement the 1D FFT on the Mixed Signal*
*Reshape the vector into Nr*Nd array*
*Run the FFT on the beat signal along the range bins dimension (Nr)*
*Normalize the FFT output*
*Take the absolute value of that output*
*Keep one half of the signal*
*Plot the output*
*There should be a peak at the initial position of the target*

```
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.

Mix = reshape(Mix,[Nr,Nd]);

%TODO*:
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.

fft_1D = fft(Mix,Nr);

%%TODO*:
% Take the absolute value of FFT output

fft_1D = abs(fft_1D);
fft_1D = fft_1D/max(fft_1D);

%TODO*:
% Output of FFT is double sided signal, but we are interested in only one 
% side of the spectrum.
% Hence we throw out half of the samples.

final_fft_1D  = fft_1D(1:Nr/2+1);

%plotting the range
figure ('Name','Range from First FFT')
plot(fft_1D)
axis ([0 200 0 1]);
xlabel('Range in meters')
ylabel('Normalized Amplitude')

```
![Plot for 1D FFT Range measurement](Desktop)
