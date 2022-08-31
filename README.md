# Multi-Sensor Fusion Algorithm in Localization for vehicles based on extended Kalman filter

This project is about multi-sensor fusion in assisted positioning based on extended Kalman filter. For the measurement
accuracy of different sensors in the measurement, a unified prediction model needs to be proposed to simulate the uncertainty
of these measurement data. In addition, the estimation accuracy will decrease under the condition of high non-linearity because
some filters are approximated by a first-order Taylor series in the error covariance matrix. To solve this problem, we propose a new multi-sensor fusion algorithm for localization, which can improve the final result of state estimation. Despite the high degree
of nonlinearity, modeling uncertainty and external interference, our proposed method can still provide better navigation and positioning results.

**Key words: Multi-Sensor Fusion, extended Kalman filter, state estimation, Localization**

## Building Motion Models

For motion models , systemization can be achieved by defining different levels of complexity. The linear motion model is at the low end of this definition. These models include, assuming constant velocity (CV) or constant acceleration (CA). On the other hand, these models assume linear motion, so rotation (especially yaw rate) cannot be considered, so nonlinear models are introduced. One example is constant turning rate(CT),which can simulate the basic motion change in non-linear model.

#### Constant Velocity Model

As for the CV model, we annote its space state with
$$
\vec{x}(t)=\left(\begin{array}{llll}
x & v_{x} & y & v_{y}
\end{array}\right)^{T}
$$
is a linear motion model, the linear state transition

$$
\vec{x}(t+T)=F* \vec{x}(t)
$$
is substituted by the state transition function vector
$$
F=\left[\begin{array}{llll}
1 & T & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & T \\
0 & 0 & 0 & 1
\end{array}\right]
$$

#### Constant Acceleration Model

As for the CA model, we annote its space state with
$$
\vec{x}(t)=\left(\begin{array}{llllll}
x & v_{x} & a_{x} & y & v_{y} & a_{y}
\end{array}\right)^{T}
$$
is a linear motion model, the linear state transition

$$
\vec{x}(t+T)=F* \vec{x}(t)
$$
is substituted by the state transition function vector
$$
F=\left[\begin{array}{llllll}
1 & T & \frac{T^2}{2} & 0 & 0 & 0 \\
0 & 1 & T & 0 & 0 & 0\\
0 & 0 & 1 & 0 & 0 & 0\\
0 & 0 & 0 & 1 & T & \frac{T^2}{2}\\
0 & 0 & 0 & 0 & 1 & T\\
0 & 0 & 0 & 0 & 0 & 1\\
\end{array}\right]
$$

#### Constant turning rate model

As for the CT model, we annote its space state with
$$
\vec{x}(t)=\left(\begin{array}{llll}
x & v_{x} & y & v_{y}
\end{array}\right)^{T}
$$
is a nonlinear motion model, the nonlinear state transition

$$
\vec{x}(t+T)=F(\omega_t)* \vec{x}(t)+\omega_t
$$
is substituted by the state transition function vector
$$
F\left(\omega_{t}\right)=\left[\begin{array}{cccc}
1 & \frac{\sin \left(\omega_{t} T\right)}{\omega_{t}} & 0 & -\frac{1-\cos \left(\omega_{t} T\right)}{\omega_{t}} \\
0 & \cos \left(\omega_{t} T\right) & 0 & -\sin \left(\omega_{t} T\right) \\
0 & \frac{1-\cos \left(\omega_{t} T\right)}{\omega_{t}} & 1 & \frac{\sin \left(\omega_{t} T\right)}{\omega_{t}} \\
0 & \sin \left(\omega_{t} T\right) & 0 & \cos \left(\omega_{t} T\right)
\end{array}\right]
$$

## Multi-sensor framework

If we have both local sensor data and global sensor data, we can get the most accurate location prediction. Therefore, it is necessary to establish a comprehensive multi-sensor prediction algorithm. In my simulation, I used the following algorithm.

![image-20220831121618500](https://raw.githubusercontent.com/wannain/image/main/2022/08/upgit_20220831_1661919378.png)

The idea of this algorithm is mainly based on the position data of traditional sensors, which is GPS prediction. If the accuracy requirements are met, there is no need to combine radar data. However, if the accuracy requirements are not met, we judge that the traditional forecast is invalid. We need to combine the radar data to correct the final forecast. 

<img src="https://raw.githubusercontent.com/wannain/image/main/2022/08/upgit_20220831_1661925536.png" alt="multi-sensors" style="zoom: 50%;" />

<center style="font-size:14px;color:#C0C0C0">   Figure.1     Block of multi-sensors algorithm</center>

## Results and conclusions

Based on the ground truth of the previous stage, we simulated two sensors, and their corresponding process noise is different. Here we distinguish, one is high process noise, and the other is low process noise. Then combined with the EKF method, we get the corresponding position measurement, and then in order to facilitate the visual comparison, we use the regularization method to calculate Corresponding to the distance from the real position, and then get the following Figures.

<img src="https://raw.githubusercontent.com/wannain/image/main/2022/08/upgit_20220831_1661925518.jpg" alt="lowpn_and_highpn" style="zoom: 50%;" />

<center style="font-size:14px;color:#C0C0C0">   Figure.2 Distance accuracy in Hign process noise and Low process noise</center>

<img src="https://raw.githubusercontent.com/wannain/image/main/2022/08/upgit_20220831_1661926846.jpg" alt="radar_hign_pn" style="zoom:50%;" />

<center style="font-size:14px;color:#C0C0C0">   Figure.3 Distance accuracy in Hign process noise and radar sensor</center>

<img src="https://raw.githubusercontent.com/wannain/image/main/2022/08/upgit_20220831_1661925498.jpg" alt="IMM" style="zoom: 50%;" />

<center style="font-size:14px;color:#C0C0C0">   Figure.4 Distance accuracy in Hign process noise, IMM filter and radar sensor</center>

<img src="https://raw.githubusercontent.com/wannain/image/main/2022/08/upgit_20220831_1661925567.jpg" alt="true_position" style="zoom: 50%;" />

<center style="font-size:14px;color:#C0C0C0">   Figure.5 Difference between final estimation measurement trajectory and true position</center>

## Conclusions

We propose a geometric map-assisted positioning algorithm, which uses the estimated trajectories from multiple sensors to obtain a position estimate. In addition, we propose a framework that combines position measurement information from VO and angle measurement information. Experiments show that, in the case of unconstrained, compared with traditional GPS sensors, the positioning error of this algorithm has been significantly reduced. Currently, we are conducting more experiments on our data set, and future work involves the application of localization methods in image sensors.