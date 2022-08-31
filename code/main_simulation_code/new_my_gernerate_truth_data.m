function [true_state, time_line, figs] = new_my_gernerate_truth_data
% Setting parameters
velocity = 200; % m/s
initial_velocity_theta=0; %degree
initial_acceleration_theta=0; %degree
omega = 8; % degree/s %Turn rate
acceleration = 3; % m/s/s
time_slot = 0.1; % time slot
time_line = (0:time_slot:floor(1000*time_slot)); % Total time line
figs = [];
% Setting system 
true_state=NaN(9,numel(time_line)); %True state % n = numel(A) 返回数组 A 中的元素数目 n 等同于 prod(size(A))。
true_state(:,1) = 0; %initialize true state

% Constant velocity model
segment_1 = floor(numel(time_line)/3);
vy=velocity*sind(initial_velocity_theta);
vx=velocity*cosd(initial_velocity_theta);
true_state(1,1) = 0; %location in x coordinate
true_state(2,1) = vx; %velocity in x coordinate
true_state(4,1) = 0; %location in y coordinate
true_state(5,1) = vy; %velocaity in y coordinate

for m = 2:segment_1
    state=true_state(:,m-1);
    state=[state(1:2);state(4:5)];
    updated_state = constvel(state,time_slot);
    updated_state=[updated_state(1:2);0;updated_state(3:4);0;0;0;0];
    true_state(:,m)=updated_state;
end

%Constant turn model
segment_2 = floor(2*numel(time_line)/3);
for m = segment_1+1:segment_2
    state=true_state(:,m-1);
    state=[state(1:2);state(4:5);omega];
    updated_state = constturn(state,time_slot);
    updated_state=[updated_state(1:2);0;updated_state(3:4);0;omega;0;0];
    true_state(:,m)=updated_state;
end

%Constant acceleration model
ay=acceleration*sind(initial_acceleration_theta);
ax=acceleration*cosd(initial_acceleration_theta);

for m = segment_2+1:numel(time_line)
    state=true_state(:,m-1);
    state=[state(1:2);ax;state(4:5);ay];
    updated_state=constacc(state,time_slot);
    updated_state=[updated_state;0;0;0];
    true_state(:,m)=updated_state;
end

figs = [figs figure];
plot(true_state(1,1:segment_1),true_state(4,1:segment_1),'.-');
hold on;
plot(true_state(1,segment_1+1:segment_2),true_state(4,segment_1+1:segment_2),'.-');
plot(true_state(1,segment_2+1:end),true_state(4,segment_2+1:end),'.-');
grid on;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('True Position')
axis equal;
legend('Constant Velocity', 'Constant Turn', 'Constant Acceleration')
end