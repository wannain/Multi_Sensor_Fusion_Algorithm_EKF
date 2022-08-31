## Introduction

This `Readme.md` is to instruct how to use the codes of CA1. 

## How to run this CA1 code

The total CA1 is code by Matlab 2021a. If you want to run this CA1, the first thing is that you update the latest version of matlab.

In this code folder, you will be easily to find there are two sub-folders named **main_simulation** and **plot_result** separately.

#### main_simulation_code

In code file, you will find these matlab files.

- `main_simulation.m` is the main function. It contains the simulation of sensors with different process noise. It also explore the estimation performance between radar sensor and IMM filter. Finally, it provide a multi-sensor fusion algorithm based on extended Kalman filter. Unfortunately, it may take half an hour to run this function because the part of getting the radar data is very time-consuming.
- `new_my_gernerate_truth_data.m` is the sub-function to generate the data of  true state.
- `radar.m` is the sub-function to generate the measurement position based on the radar sensor.  Unfortunately, it may take a lot of time to run this function owing to the full scan in target estimation in radar sensors.
- `radar.mat` is the data I collect based on  radar sensor and save it as `.mat` file. It contains the position estimation based on the radar detection system.
- `load_data_main_simulation.m` is the main function without the generation radar estimation. So if you update the correct path location of `radar.mat`, it is very easy and quick to get the final result. If you want to run this function, the only thing to do is that you should update the correct path of `radar.mat`. The location is in the **line No.95 ** of this code. After loading data well, you will get the final result by running this code. 

More detailed comment is in the code, so you can check it.



#### plot_result

In code file, you will find these pictures .

- `Lowpn_and_highpn.jpg` is the picture that shows the distance estimation accuracy in sensors between hign process noise and low process noise.
- `Radar_hign_pn.jpg` is the picture that shows the distance estimation accuracy in sensors with hign process noise and that of radar sensors.
- `IMM.jpg` is the picture that shows the distance estimation accuracy in sensors with hign process noise , radar sensors and IMM filter sensors.
- `Mulit_sensors.jpg` is the picture that shows the difference between final estimation trajectory based on mulit-sensors algorithm and true position.

More detailed comment is in the report, so you can check it.

