clc
clear all
close all
[true_state, time, fig1] = new_my_gernerate_truth_data;
dt = diff(time(1:2));
num_steps = numel(time);
%figure(fig1)

s = rng;
rng(2021);
position_selector = [1 0 0 0 0 0;0 0 1 0 0 0;0 0 0 0 1 0]; % Position from state
%true_position = position_selector * true_state;
true_position = [true_state(1,:);true_state(4,:);true_state(7,:)];
%true_position=true_state;
measurement_noise = randn(size(true_position));
measurement_position = true_position + measurement_noise;

initial_state = position_selector' * measurement_position(:,1); %First state data from measurement
initial_covariance = diag([1,1e4,1,1e4,1,1e4]); % Velocity is not measured
cvekf = trackingEKF(@constvel, @cvmeas, initial_state, ...
    'StateTransitionJacobianFcn', @constveljac, ...
    'MeasurementJacobianFcn', @cvmeasjac, ...
    'StateCovariance', initial_covariance, ...
    'HasAdditiveProcessNoise', false, ...
    'ProcessNoise', eye(3));

dist_1 = zeros(1,num_steps); %Low Process Noise
estimate_position = zeros(3,num_steps); %Estimated position
for i = 2:size(measurement_position,2)
    predict(cvekf, dt);
    dist_1(i) = distance(cvekf,true_position(:,i)); % Distance from true position
    estimate_position(:,i) = position_selector * correct(cvekf, measurement_position(:,i));
end
figure(fig1);
plot(estimate_position(1,:),estimate_position(2,:),'.g','DisplayName','CV Low PN')
title('True and Estimated Positions')
%axis([5000 8000 -500 2500])

fig2 = figure;
hold on
plot((1:num_steps)*dt,dist_1,'g','DisplayName', 'CV Low PN')
title('Normalized Distance From Estimated Position to True Position')
xlabel('Time (s)')
ylabel('Normalized Distance')
legend

cvekf2 = trackingEKF(@constvel, @cvmeas, initial_state, ...
    'StateTransitionJacobianFcn', @constveljac, ...
    'MeasurementJacobianFcn', @cvmeasjac, ...
    'StateCovariance', initial_covariance, ...
    'HasAdditiveProcessNoise', false, ...
    'ProcessNoise', diag([50,50,1])); % Large uncertainty in the horizontal acceleration
dist_2 = zeros(1,num_steps); %Hign Process Noise
estimate_position_2 = zeros(3,num_steps);
for i = 2:size(measurement_position,2)
    predict(cvekf2, dt);
    dist_2(i) = distance(cvekf2,true_position(:,i)); % Distance from true position
    estimate_position_2(:,i) = position_selector * correct(cvekf2, measurement_position(:,i));
end
figure(fig1)
plot(estimate_position_2(1,:),estimate_position_2(2,:),'.c','DisplayName','CV High PN')

figure(fig2)
plot((1:num_steps)*dt,dist_2,'c','DisplayName', 'CV High PN')
axis([0 100 0 900])

% Use an Interacting Motion-Model Filter
imm = trackingIMM('TransitionProbabilities', 0.99); % The default IMM has all three models
% Initialize the state and state covariance in terms of the first model
initialize(imm, initial_state, initial_covariance);
dist_IMM = zeros(1,num_steps);
estimate_position_IMM = zeros(3,num_steps);
modelProbs = zeros(3,num_steps);
modelProbs(:,1) = imm.ModelProbabilities;
for i = 2:size(measurement_position,2)
    predict(imm, dt);
    dist_IMM(i) = distance(imm,true_position(:,i)); % Distance from true position
    estimate_position_IMM(:,i) = position_selector * correct(imm, measurement_position(:,i));
    modelProbs(:,i) = imm.ModelProbabilities;
end
figure(fig1)
plot(estimate_position_IMM(1,:),estimate_position_IMM(2,:),'.m','DisplayName','Multi-sensor')
initialize(imm, initial_state, initial_covariance);

figure(fig2)
hold on
plot((1:num_steps)*dt,dist_IMM,'m','DisplayName', 'Multi-sensor')
axis([0 100 0 900])

%Use a radar sensor to get estimation 
dist_radar = zeros(1,num_steps);
%estimate_position_radar = zeros(3,num_steps);
radar_estimated_position = zeros(3,num_steps);
radar_estimated_position_data=zeros(3,num_steps);
load radar.mat
for i = 1:size(measurement_position,2)
    %radar_state = position_selector' * measurement_position(:,i);
    %radar_state= measurement_position(:,i);
    radar_state= true_state(:,i);
    radar_estimated_position_data(:,i)=radar_estimated_position(:,i);
    %this_radar_estimated_position=radar(radar_state);
    %radar_estimated_position(:,i)=this_radar_estimated_position;
    dist_radar(i) = norm(radar_estimated_position_data(:,i)-true_position(:,i)); % Distance from true position
end
figure(fig1)
plot(radar_estimated_position_data(1,:),radar_estimated_position_data(2,:),'.c','DisplayName','CV in radar estimation')
figure(fig2)
hold on
plot((1:num_steps)*dt,dist_radar,'DisplayName', 'radar','MarkerEdgeColor','k')
axis([0 100 0 900])