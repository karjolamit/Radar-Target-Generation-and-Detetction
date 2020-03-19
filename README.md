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
    
    %For each time stamp update the Range of the Target for constant 
    %velocity. 
    r_t(i) = R + t(i)*v;
    td(i) = r_t(i)*2/c;
    
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos(2*pi*(fc*t(i)+slope*t(i)^2*0.5));
    Rx(i) = cos(2*pi*(fc*(t(i)-td(i))+slope*(t(i)-td(i))^2*0.5));
    
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i).*Rx(i);
    
end
```

## Range Measurement - 1st FFT Operation
*1. Implement the 1D FFT on the Mixed Signal*
2. *Reshape the vector into Nr*Nd array*
3. *Run the FFT on the beat signal along the range bins dimension (Nr)*
4. *Normalize the FFT output*
5. *Take the absolute value of that output*
6. *Keep one half of the signal*
7. *Plot the output*
8. *There should be a peak at the initial position of the target*

```
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.

Mix = reshape(Mix,[Nr,Nd]);

%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.

fft_1D = fft(Mix,Nr);

% Take the absolute value of FFT output

fft_1D = abs(fft_1D);
fft_1D = fft_1D/max(fft_1D);

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
![Plot for 1D FFT Range measurement](https://github.com/karjolamit/Radar-Target-Generation-and-Detetction/blob/master/Plot%20for%201D%20FFT%20Range%20measurement.png)

From above figure, we see the target range is 140 meters; which is same as we set it in the target specifications (R = 140)

## Range Doppler Map Generation - 2nd FFT Operation

```
% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix = reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift(sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM);

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
%figure,surf(doppler_axis,range_axis,RDM);
figure ('Name','surface plot of FFT2')
surf(doppler_axis,range_axis,RDM);
title('FFT2 surface plot')
xlabel('Speed in m/s')
ylabel('Range in meters')
zlabel('Amplitude')
```
![Plot for 2D FFT Range Doppler Map](https://github.com/karjolamit/Radar-Target-Generation-and-Detetction/blob/master/Plot%20for%202D%20FFT%20Range%20Doppler%20Map.png)

## 2D CFAR Operation
*Determine the number of Training cells for each dimension Tr and Td. Similarly, pick the number of guard cells Gr and Gd.
Slide the Cell Under Test (CUT) across the complete cell matrix
Select the grid that includes the training, guard and test cells 
Grid Size = (2Tr+2Gr+1) * (2Td+2Gd+1)
The total number of cells in the guard region and cell under test. (2Gr+1) * (2Gd+1)
This gives the Training Cells: (2Tr+2Gr+1) * (2Td+2Gd+1) - (2Gr+1) * (2Gd+1)
Measure and average the noise across all the training cells. This gives the threshold
Add the offset (if in signal strength in dB) to the threshold to keep the false alarm to the minimum
Determine the signal level at the Cell Under Test
If the CUT signal level is greater than the Threshold, assign a value of 1, else equate it to zero
Since the cell under test are not located at the edges, due to the training cells occupying the edges, we suppress the edges to zero
Any cell value that is neither 1 nor a 0, assign it a zero*

![FFT block cells](https://github.com/karjolamit/Radar-Target-Generation-and-Detetction/blob/master/FFT%20block%20cells.png)

The CFAR process includes the sliding of a window across the cells in FFT blocks. Each window consists of the following cells:

**Cell Under Test (CUT):** The cell that is tested to detect the presence of the target by comparing the signal level against the noise estimate (threshold).
**Training Cells:** The level of noise is measured over the Training Cells. The Training Cells can be divided into two regions, the cells lagging the CUT, called lagging Training Cells and the cells leading the CUT, called Leading Training Cells. 
The number of training cells is decided based on the environment. If a dense traffic scenario then the fewer training cells should be used, as closely spaced targets can impact the noise estimate. Considering this, I selected the following values for training cells.
**Guard Cells:** The cells just next to CUT are assigned as Guard Cells. The purpose of the Guard Cells is to avoid the target signal from leaking into the training cells that could adversely affect the noise estimate. The number of guard cells should be decided based on the leakage of the target signal out of the cell under test. If target reflections are strong, they often get into surrounding bins.
**Threshold Factor (Offset):** Use an offset value to scale the noise threshold. If the signal strength is defined in logarithmic form then add this offset value to the average noise estimate, else multiply it.

```
%Select the number of Training Cells in both the dimensions.

Tr = 20;
Tc = 8;

%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation

Gc = 6;
Gr = 12;

% offset the threshold by SNR value in dB

offset = 3;

%Create a vector to store noise_level for each iteration on training cells

noise_level = zeros(1,1);

%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.


   % Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
   % CFAR
   
   RDM = RDM/max(max(RDM));
   [row,col] = size(RDM);
   CUT = zeros(row,col);
    
   for i = Tr+Gr+1:(Nr/2-Tr-Gr)
       for j = Tc+Gc+1:(Nd-Tc-Gc)
           noise_level = zeros(1,1);
           for k = (i-Tr-Gr):(i+Tr+Gr)
               for h = (j-Tc-Gc):(j+Gc+Tc)
                   if(abs(k-i)>Gr||abs(h-j)>Gc)
                       noise_level = noise_level + db2pow(RDM(k,h));
                   end
               end
           end
           length = 2*(Tr+Gr)+1;
           width =  2*(Tc+Gc)+1;
           threshold = pow2db(noise_level/(length*width-(2*Gr+1)*(2*Gc+1)))*1.4;
           if(RDM(i,j)>threshold)
               CUT(i,j) = 1;
           else
               CUT(i,j) = 0;
           end
       end
   end


%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
% figure,surf(doppler_axis,range_axis,'replace this with output');
% colorbar;

figure('Name', 'CA-CFAR Filtered RDM')
surf(doppler_axis,range_axis,CUT);
colorbar;
title('CA-CFAR Filtered RDM surface plot');
xlabel('Speed in m/s');
ylabel('Range in meters');
zlabel('Normalized Amplitude');

figure('Name', 'CA-CFAR Filtered Range')
surf(doppler_axis,range_axis,CUT);
colorbar;
title('CA-CFAR Filtered RDM surface plot');
xlabel('Speed in m/s');
ylabel('Range in meters');
zlabel('Normalized Amplitude');
view(90,0);

figure('Name', 'CA-CFAR Filtered Speed')
surf(doppler_axis,range_axis,CUT);
colorbar;
title( 'CA-CFAR Filtered RDM surface plot');
xlabel('Speed in m/s');
ylabel('Range in meters');
zlabel('Normalized Amplitude');
view(0,0);
```

![CA-CFAR Range Doppler Measurement plot](https://github.com/karjolamit/Radar-Target-Generation-and-Detetction/blob/master/CA-CFAR%20Range%20Doppler%20Measurement%20plot.png)

![CA-CFAR Range Measurement plot](https://github.com/karjolamit/Radar-Target-Generation-and-Detetction/blob/master/CA-CFAR%20Range%20Measurement%20plot.png)

![CA-CFAR Doppler Measurement plot](https://github.com/karjolamit/Radar-Target-Generation-and-Detetction/blob/master/CA-CFAR%20Doppler%20Measurement%20plot.png)
