% Localization using a grid of magnets in the ground.
% -----
% Usage: 
%    - Set the characteristics of the robot and sensor in 
%      RobotAndSensorDefinition.m
%    - Set noise levels in DefineVariances.m
%    - Set robot initial position in the present file.
%    - Execute this file.
%    - Then, execute PlotResults.m to get the plots for analysis.
% You can also use, for a brand new fresh start each time:
% clear all; close all; MagnetLoc; PlotResults;
% -----
% Project history:
%    - Project initiator and principle of the sensor: Gaëtan Garcia
%    - Sensor manufacturing and test: Joël Rousseau.
%    - Characterization of the sensor: EMARO1/ARIA1 project, Hendry Chame
% and Marcelo Gaudenzi de Faria. Supervision: Gaëtan Garcia
%    - First implementation (Matlab): EMARO1/ARIA1 project, Filip Cichosz
% and Marteen Samuel. Supervision: Gaëtan Garcia.
%    - This program (using Samuel and Cichosz's work): Gaëtan Garcia

RobotAndSensorDefinition ;
DefineVariances ;

X = [ 0, 0, 0*pi/180 ].' ;    % Set this according to robot initial position.

%Load the data file
dataFile = uigetfile('*.mat','Select data file') ;
if isunix 
    load(dataFile);
else
    load(dataFile);
end

P = Pinit ; 

% Log data for display of results. Do not modify.

LogData( 0 , 'init' , X , P , [0;0] ) ;

% Skip motionless parts of the data at beginning and end of the experiment
% and return only meaningful data, with wheel rotations in radians.
% Also reduce encoder resolution and frequency according to factors
% set in RobotDefinition.m

wbHandle = waitbar(0,'Computing...') ;

for i = 2 : length(treal) 
    
    t = (i-1)*samplingPeriod ;
    
    waitbar(i/length(treal)) ;

    % Calculate input vector from proprioceptive sensors
    deltaq = [ qR(i) - qR(i-1) ; 
               qL(i) - qL(i-1) ] ;
    U = jointToCartesian * deltaq ;  % joint speed to Cartesian speed.
    
    % Predic state (here odometry)
    X = EvolutionModel( X , U ) ;
    
    % Calculate linear approximation of the system equation
    A = [ 1 0 -U(1)*sin(X(3));
          0 1 U(1)*cos(X(3)) ;
          0  0   1] ;
    B = [ cos(X(3)) 0;
          sin(X(3)) 0;
          0 1] ;
       
    % Error propagation
    P = A*P*(A.') + B*Qbeta*(B.') + Qalpha ;
    
    LogData( t , 'prediction' , X , P , U , [0;0] ) ;
    
    % Vector of measurements. Size is zero if no magnet was detected.

    measures = sensorState(i,:) ;
            

    for measNumber = 1 : length(measures) 
        if measures(measNumber)==1 
            % Homogeneous transform of robot frame with respect to world frame
            oTm = [ cos(X(3)) -sin(X(3)) X(1);
                    sin(X(3))  cos(X(3)) X(2);
                    0   0   1] ;
            mTo = inv(oTm) ;

            % Measurement vector: coordinates of the sensor in Rm.
            Y = mSensors(1:2,measNumber);

            % Now in homogeneous coordinates for calculations.
            mMeasSensor = mSensors(:,measNumber);

            % Corresponding position in absolute frame. 
            oMeasSensor = oTm * mMeasSensor ;
            oMeasSensorNormalized=oMeasSensor./[xSpacing;ySpacing;1];
            if oMeasSensorNormalized(1)>0.5
                oMeasSensorNormalized(1)=1-oMeasSensorNormalized(1);
            end
            if oMeasSensorNormalized(2) >0.5
                oMeasSensorNormalized(2)=1-oMeasSensorNormalized(2);
            end
            if oMeasSensorNormalized(1)<oMeasSensorNormalized(2) %une verticale est plus proche qu'une horizontale 
                
                % Due to measurement and localization errors, the previously calculated
                % position does not match an actual magnet.
                % Which actual magnet is closest? It will be the candidate magnet for
                % the update phase of the state estimator.
                oRealSensor=oMeasSensor;
                oRealSensor(1) = round(oMeasSensor(1)/xSpacing)*xSpacing ;

                % The position of the real magnet in robot frame. It will in general 
                % be different from the measured one. 
                mRealSensor = oTm \ oRealSensor ;  % That's inv(oTm)*oRealMagnet = mTo*oRealMagnet

                % The expected measurement are the two coordinates of the real 
                % magnet in the robot frame.
                Yhat = mRealSensor(1:2) ;

                C = [ 1 0 -mSensors(measNumber,1)*sin(X(3))-mSensors(measNumber,2)*cos(X(3))];

                innov = Y - Yhat ;   
                dMaha = sqrt( innov(1) / ( C*P*C.' + Qgamma(1,1)) * innov(1) ) ;

                LogData( t , 'measurement' , X , P , [0;0] , Y ) ;

                if dMaha <= mahaThreshold
                    K = P * C.' * inv( C*P*C.' + Qgamma(1,1)) ;
                    X = X + K*innov(1) ;
                    P = (eye(length(X)) - K*C) * P ;
                    LogData( t , 'update' , X , P , [0;0] , [0;0] ) ;
                end
            else %ligne horizontale
                
                % Due to measurement and localization errors, the previously calculated
                % position does not match an actual magnet.
                % Which actual magnet is closest? It will be the candidate magnet for
                % the update phase of the state estimator.
                oRealSensor=oMeasSensor;
                oRealSensor(2) = round(oMeasSensor(2)/ySpacing)*ySpacing ;

                % The position of the real magnet in robot frame. It will in general 
                % be different from the measured one. 
                mRealSensor = oTm \ oRealSensor ;  % That's inv(oTm)*oRealMagnet = mTo*oRealMagnet

                % The expected measurement are the two coordinates of the real 
                % magnet in the robot frame.
                Yhat = mRealSensor(1:2) ;

                C = [ 0 1 mSensors(measNumber,1)*cos(X(3))-mSensors(measNumber,2)*sin(X(3))];

                innov = Y - Yhat ;   
                dMaha = sqrt( innov(2) / ( C*P*C.' + Qgamma(2,2)) * innov(2) ) ;

                LogData( t , 'measurement' , X , P , [0;0] , Y ) ;

                if dMaha <= mahaThreshold
                    K = P * C.' * inv( C*P*C.' + Qgamma(2,2)) ;
                    X = X + K*innov(2) ;
                    P = (eye(length(X)) - K*C) * P ;
                    LogData( t , 'update' , X , P , [0;0] , [0;0] ) ;
                end
            end
        end
    end
end

% Save all tunings and robot parameters, so the conditions under
% which the results were obtained are known. Also save inputs and 
% measurements for later display.    
    
save inputLog ...
     rwheel trackGauge encoderRes samplingPeriod ...
     xSpacing ySpacing  ...
     U sensorState ...
     Pinit Qgamma sigmaTuning Qbeta Qalpha mahaThreshold 

 
LogData( t , 'termination' , X , P , [0;0] , [0;0] ) ;
close(wbHandle) ;
%close all;
PlotResults;