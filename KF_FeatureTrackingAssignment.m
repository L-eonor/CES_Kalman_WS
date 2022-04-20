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

%% State Transition Matrix

dt=0.025; % Time between measurements

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              
%A = ... ; % uncomment this line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Observation Model

% Only position measurements (x,y) are available

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              
%C = ... ; % uncomment this line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Modelling Noise

% Measurement noise -> sensor uncertainty
MeasurementStd=0.0030;
CovMeasurements = MeasurementStd^2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              
%R = ... ; % uncomment this line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Process Noise -> Acceleration modelled as process external interference
AccelStd = 250;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              
%AccelVar = ... ; % uncomment this line
%Q = ... ;        % uncomment this line
%P = ... ;        % uncomment this line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

    % Predict next Covariance matrix
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % YOUR CODE HERE %%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
    %P = ... ; % uncomment this line
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% Update Step
    % Update Kalman Gain
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % YOUR CODE HERE %%%%%%%%%%%%%%%
    % Hint:Measurements may be incomplete
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
    %K = ... ; % uncomment this line
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Update State Vector
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % YOUR CODE HERE %%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
    %StateVector = ... ; % uncomment this line
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Update Covariance
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % YOUR CODE HERE %%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
    %P = ... ; % uncomment this line
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
for i=1:NumSamples
    frame1=vid(:,:,:,i+7);
    imshow(frame1);
    hold on
    measPlot=scatter(MeasuredPosition(1,i),MeasuredPosition(2,i),'g','filled');
    truthPlot=scatter(GroundTruthPosition(1,i),GroundTruthPosition(2,i),'r','filled');
    estPlot=scatter(PositionEstimate(1,i),PositionEstimate(2,i),'b','filled');
    legend([measPlot, truthPlot, estPlot],{'Measurement','Ground Truth','KF Estimation'});
    pause(0.025);
    delete(measPlot);
    delete(truthPlot);
    delete(estPlot);
end