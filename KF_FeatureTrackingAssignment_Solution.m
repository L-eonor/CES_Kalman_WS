%% Assignment: Dummy Head Feature Tracking using Linear Kalman Filter
close all
clear all
% Read dataset
GroundTruth=readtable('FeaturePointDataset_GT.csv');
Measurements=readtable('FeaturePointDataset_someNoise.csv');
%Measurements=readtable('FeaturePointDataset_tonsNoise.csv');
%Measurements=readtable('FeaturePointDataset_incomplete.csv');

sizeData=size(Measurements);
NumSamples=sizeData(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Assignment %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% System State Vector
xInit=Measurements(1,:).x;    
yInit=Measurements(1,:).y;    
x_dotInit = 0;                
y_dotInit = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%             
%StateVector = ... ; % uncomment this line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
StateVector = [xInit;yInit;0;0];

%% State Transition Matrix

dt=0.025; % Time between measurements

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              
%A = ... ; % uncomment this line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];

%% Observation Model

% Only position measurements (x,y) are available

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              
%C = ... ; % uncomment this line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C=[1 0 0 0; 0 1 0 0];

%% Modelling Noise

% Measurement noise -> sensor uncertainty
MeasurementStd=0.003;
CovMeasurements = MeasurementStd^2;%0.00001;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              
%R = ... ; % uncomment this line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
R=[CovMeasurements,0;0,CovMeasurements];

% Process Noise -> Acceleration modelled as process external interference
AccelStd = 250;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              
%AccelVar = ... ; % uncomment this line
%Q = ... ;        % uncomment this line
%P = ... ;        % uncomment this line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
AccelVar = AccelStd^2;
G=[(dt^2)/2 0; 0 (dt^2)/2;  dt 0; 0 dt];
Q=[(dt^4)/4   0         (dt^3)/2   0;        ...
     0         (dt^4)/4  0          (dt^3)/2; ...
    (dt^3)/2   0         dt^2       0;        ...
     0         (dt^3)/2  0          dt^2];
P = Q.*AccelVar;
%% Kalman Model

%Initializations
vel = [];
PositionEstimate = [];
VelocityEstimate = [];

for t=1:NumSamples
    % Obtain our measured position for cycle t
    MeasuredPosition(:,t) = [Measurements(t,:).x ; Measurements(t,:).y];
    GroundTruthPosition(:,t) = [GroundTruth(t,:).x ; GroundTruth(t,:).y];

    %% Prediction Step

    % Predict next state
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % YOUR CODE HERE %%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%             
    %StateVector = ... ; % uncomment this line
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    StateVector= A * StateVector + Q.*AccelVar;

    % Predict next Covariance matrix
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % YOUR CODE HERE %%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
    %P = ... ; % uncomment this line
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    P = A*P*A' + Q;

    %% Update Step

    % Update Kalman Gain
    
    if ~(isnan(MeasuredPosition(:,t)))
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		% YOUR CODE HERE %%%%%%%%%%%%%%%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
		%K = ... ; % uncomment this line
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		K = P*C'*inv(C*P*C' + R);
	
		% Update State Vector
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		% YOUR CODE HERE %%%%%%%%%%%%%%%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
		%StateVector = ... ; % uncomment this line
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        StateVector = StateVector + K*(MeasuredPosition(:,t) - C * StateVector);

		% Update Covariance
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		% YOUR CODE HERE %%%%%%%%%%%%%%%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
		%P = ... ; % uncomment this line
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		P = P-K*C*P;
    end

    % Store Data
    PositionEstimate=[PositionEstimate; StateVector(1:2)];
    VelocityEstimate= [VelocityEstimate; StateVector(3:4)];

end

%% Plots

PositionEstimate=PositionEstimate';
VelocityEstimate=VelocityEstimate';
% read the total number of frames
obj = VideoReader('DummyTrackingVideo.avi');
vid=read(obj);
frames = obj.NumberOfFrames;
%v = VideoWriter('example.avi');
%open(v);

for i=1:NumSamples
    frame1=vid(:,:,:,i+7);
    imshow(frame1);
    hold on
    measPlot=scatter(MeasuredPosition(1,i),MeasuredPosition(2,i),'g','filled');
    truthPlot=scatter(GroundTruthPosition(1,i),GroundTruthPosition(2,i),'r','filled');
    estPlot=scatter(PositionEstimate(1,i),PositionEstimate(2,i),'b','filled');
    legend([measPlot, truthPlot, estPlot],{'Measurement','Ground Truth','KF Estimation'});
    %frame = getframe(gcf);
    %writeVideo(v,frame);
    pause(0.025);
    delete(measPlot);
    delete(truthPlot);
    delete(estPlot);
end
%close(v);