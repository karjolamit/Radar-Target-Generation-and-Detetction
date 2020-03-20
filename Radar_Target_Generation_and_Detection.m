clear all
clc;

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

%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant

R = 140;                     % target's initial position
v = 40;                      % target's velocity
 
%% FMCW Waveform Generation

% *%TODO* :
% Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of 
% the FMCW chirp using the requirements above.

B_sweep = c/2*d_res;         % Sweep Bandwidth
T_chirp = 5.5*2*R/c;         % Chirp Time
slope = B_sweep/T_chirp;     % Slope of FMCW

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

%% RANGE MEASUREMENT

%TODO*:
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
%subplot(2,1,1)

%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM

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

%% CFAR implementation

%Slide Window through the complete Range Doppler Map

%TODO*:
%Select the number of Training Cells in both the dimensions.

Tr = 20;
Tc = 8;

%TODO*:
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation

Gc = 6;
Gr = 12;

%TODO*:
% offset the threshold by SNR value in dB

offset = 3;

%TODO*:
%Create a vector to store noise_level for each iteration on training cells

noise_level = zeros(1,1);

%TODO*:
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

%TODO*:
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 
% Not required 

%TODO*:
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
