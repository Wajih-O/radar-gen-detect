clear all
clc;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
maxRange = 200
% Range Resolution = 1 m
rangeRes = 1
% Max Velocity = 100 m/s
maxV = 100 % used as constant in the original script (RANGE DOPPLER RESPONSE impl. section)
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Speed of light
c = 3e8;

%% User Defined Range and Velocity of target
% define the target's initial position and velocity. Note : Velocity
% remains contant

R_init = 90 % vehicle range
v=10 % vehicle velocity


%% FMCW Waveform Generation

%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.

B = c/(2*rangeRes);
Tchirp = 5.5*2*maxRange/c; % sweep time
slope = B/Tchirp;

%Operating carrier frequency of Radar 
fc= 77e9;             %carrier freq

                                                          
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*Tchirp,Nr*Nd); %total time for samples


%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 


for i=1:length(t)         
    
    %For each time stamp update the Range of the Target for constant velocity. 
    r_t(i) = R_init + (t(i)*v);
    td(i) = t(i) - (2*r_t(i)/c);
    
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos(2*pi*(fc*t(i) + (slope * (t(i)^2)/2)));
    Rx (i)  = cos(2*pi*(fc*td(i) + (slope * (td(i)^2)/2)));
    
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i)*Rx(i);
    
end


%% RANGE MEASUREMENT

%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
MixReshaped = reshape(Mix, [Nr,Nd]);

%Range and Doppler FFT respectively.

%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
sig_fft = fft(MixReshaped, Nr)./Nr;
% Take the absolute value of FFT output
sig_fft = abs(sig_fft);

disp('sig fft')
size(sig_fft)


% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
sig_fft = sig_fft(1:Nr/2);

%plotting the range
figure ('Name','Range from First FFT')
subplot(2,1,1)

 % plot FFT output 
plot(sig_fft)
axis ([0 200 0 1]);




%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);

%% CFAR implementation

% Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
% CFAR

%--------------
% CFAR params
% the number of Training Cells in both the dimensions.
Tr = 20; % training cells (range)
Td = 20; % training cells (dopler)

%Select the number of Guard Cells in both dimensions around the Cell under 
Gr = 4;  % number of Guard Cells (range)
Gd = 4;  % number of Guard Cells (dopler)
offset = 5 % offset the threshold by SNR value in dB
%--------------

training_cells_nbr = (2*(Tr+Gr)+1)*(2*(Td+Gd)+1) - ((2*Gr+1)*(2*Gd+1));  % Get only the util cardinality (training cell)

output = zeros(size(RDM));

% The process below generate a thresholded block, which is smaller 
% than the Range Doppler Map as the CUT cannot be located at the edges of
% matrix. Hence,few cells will not be thresholded.

% The previous initialization of output keeps the map size same set those values to 0.

% A loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.

for i = Tr+Gr+1:Nr/2-(Tr+Gr)
    for j = Td+Gd+1:Nd-(Td+Gd)
        %Slide Window through the complete Range Doppler Map
        TGCrop = RDM(i-(Gr+Tr):i+Gr+Tr, j-(Gd+Td):j+Gd+Td); % T + G + cut a context crop 
        GCrop = zeros(size(TGCrop));
        GCrop(Tr+1:Tr+2*Gr+1, Td+1:Td+2*Gd+1) = RDM(i-Gr:i+Gr,j-Gd:j+Gd); % G + cut crop
        
        util = TGCrop - GCrop; % Extracts util region (Training cells)
        % size(util(util > 0))(0) == training_cells_nbr % a sanity check of
        % training cells nbr
        
        % Sum convert the value from logarithmic to linear using db2pow
        % function. Average the summed values for all of the training
        %cells used and convert it back to logarithimic using pow2db.
        noise_level =  pow2db(sum(db2pow(util(util > 0))) / training_cells_nbr);
        
        % Add the offset to it to determine the threshold 
        % If the CUT level > threshold assign it a value of 1, else equate it to 0.
        output(i, j) = (RDM(i, j) > (noise_level + offset));
    end
end

%Display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure,surf(doppler_axis,range_axis,output);
colorbar;


 
 