
# Radar Target Generation and Detection

## Introduction

This project simulates a moving target radar detection. The target moves with a constant velocity model and within the range of the Radar specifications (Max range and velocity). The complete simulation and detection process/pipeline is implemented in  the `radar_target_generation_and_detection.m` Matlab file.The `Section and layout` documents  the project's components and how they relate to the code sections.

## Section and layout

* Configuration of the FMCW waveform based on the system requirements. (`FMCW Waveform Generation` code section)
* Defining the range and velocity of target and simulate its displacement. (` User Defined Range and Velocity of target` code section)
* Transmit and receive signal to determine the beat signal. (`Signal generation and Moving Target simulation` section)

* Perform Range FFT on the received signal to determine the Range (`RANGE MEASUREMENT` and `RANGE DOPPLER RESPONSE` code sections)

* Towards the end, a CFAR processing on the output of 2nd FFT to display the target. (`CFAR implementation` code section)

<img src="images/project_layout.png" width="620" />

## CFAR Output example/instance
For a generated range of 90 and velocity of 10 m/s

```matlab
R_init = 90 % vehicle range
v=10 % vehicle velocity
```

and CFAR parameters

```matlab
Tr = 20; % training cells (range)
Td = 20; % training cells (dopler)

%Select the number of Guard Cells in both dimensions around the Cell under
Gr = 4;  % number of Guard Cells (range)
Gd = 4;  % number of Guard Cells (dopler)
offset = 5 % offset the threshold by SNR value in dB
```
we obtain the following output

<img src="images/CFAR_output.png"/>