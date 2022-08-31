function radar_estimated_position=radar(state)
%state=[20,200,0,0,0,0,1000,0,0]';
%function radar_estimated_position=radar()
position_state=[state(1,:) state(4,:) state(7,:)];
velocity_state=[state(2,:) state(5,:) state(8,:)];
%clc
%clear all
%close all
%position_state=[20 0 0];
%velocity_state=[200 0 0];
%s = rng;
%rng(2021);
%position_state=[10000 -50e3 -1e3];
%velocity_state=[50 900*1e3/3600 0];
tgt1 = struct('PlatformID',1, ...
    'Position',position_state, ...
    'Velocity',velocity_state);
rpm = 12.5;
fov = [1.4; 5]; % [azimuth; elevation]
scanrate = rpm*360/60;  % deg/s
updaterate = scanrate/fov(1); % Hz

sensor = fusionRadarSensor(1,'Rotator', ...
    'UpdateRate',updaterate, ...
    'MountingLocation',[0 0 -15], ...
    'MaxAzimuthScanRate',scanrate, ...
    'FieldOfView',fov, ...
    'AzimuthResolution',fov(1));
simTime = 0;
detBuffer = {};
while true
    [dets,numDets,config] = sensor([tgt1],simTime);
    detBuffer = [detBuffer; dets]; %#ok<AGROW>

    % Is full scan complete?
    if config.IsScanDone
        measurement_position_noise = randn(size(position_state));
        detPos = position_state + measurement_position_noise;
        break % yes
    end
    simTime = simTime + 1/sensor.UpdateRate;
end

radarPosition = [0 0 0];
tgtPositions = [tgt1.Position];
clrs = lines(3);

%figure
%hold on

% Plot radar position
%plot3(radarPosition(1),radarPosition(2),radarPosition(3),'Marker','s', ...'DisplayName','Radar','MarkerFaceColor',clrs(1,:),'LineStyle','none')

% Plot truth
%plot3(tgtPositions(:,1),tgtPositions(:,2),tgtPositions(:,3),'Marker','^', ... 'DisplayName','Truth','MarkerFaceColor',clrs(2,:),'LineStyle', 'none')

% Plot detections
%if ~isempty(detBuffer)
    %detPos = cellfun(@(d)d.Measurement(1:3),detBuffer, ...
        %'UniformOutput',false);
    %detPos = cell2mat(detPos')';
    %plot3(detPos(:,1),detPos(:,2),detPos(:,3),'Marker','o', ...
        %'DisplayName','Detections','MarkerFaceColor',clrs(3,:),'LineStyle','none')
%end

%xlabel('X(m)')
%ylabel('Y(m)')
%axis('equal')
%legend
radar_estimated_position=detPos;
radar_estimated_position=radar_estimated_position';
%return radar_estimated_position
end