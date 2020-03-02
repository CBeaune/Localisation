% Main function of the Localisation Project, using the tiles' joints

% -----
% Usage: 
%    - Set the characteristics of the robot and sensor in 
%      RobotAndSensorDefinition.m
%    - Set noise levels in DefineVariances.m
%    - Set robot initial position in the present file.
%    - Execute CreateRobotTraj.m and SimulateSensor.m if you want to create
%    a specific trajectory.
%    - Execute this file.
%    - Then, execute PlotResults.m to get the plots for analysis.
% -----
% Project history:
%    - Inspired by the work on localisation based on magnets by : Gaetan
%    Garcia, Joel Rousseau, Hendry Chame, Marcelo Gaudenzi de Faria, Filip 
%    Cichosz, Marteen Samuel.
%    - Adapted on tiles' joints by : Charlotte Beaune, Audrey Boyenval,
%    Alessandro Masi

%%
clear all;
RobotAndSensorDefinition ;
DefineVariances ;

%% Load the data file
dataFile = uigetfile('*.mat','Select data file') ;
if isunix 
    load(dataFile);
else
    load(dataFile);
end
%% Initial position
X = [ xreal(1),
    yreal(1),
    atan2( (yreal(2)-yreal(1)) , (xreal(2)-xreal(1)) ) ].' ;
P = Pinit ; 

%% Log data for display of results.

LogData( 0 , 'init' , X , P , [0;0] ) ;

%%
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
    U = jointToCartesianfaux * deltaq ;  % joint speed to Cartesian speed.
    
    % Predic state (here odometry)
    X = EvolutionModel( X , U ) ;
    
    % Calculate linear approximation of the system equation
    A = [ 1 0 -U(1)*sin(X(3));
          0 1 U(1)*cos(X(3)) ;
          0 0 1] ;
      
    B = [ cos(X(3)) 0;
          sin(X(3)) 0;
          0         1] ;
       
    % Error propagation
    P = A*P*(A.') + B*Qbeta*(B.') + Qalpha ;
    
    LogData( t , 'prediction' , X , P , U , [0;0] ) ;
    
    % Vector of sensor data at time t
    measures = sensorState(i,:) ; 
 
    for measNumber = 1 : length(measures) 
        if measures(measNumber)==lineDetected 
            % Homogeneous transform of robot frame with respect to world frame
            oTm = [ cos(X(3)) -sin(X(3)) X(1);
                    sin(X(3))  cos(X(3)) X(2);
                    0   0   1] ;

            % Now in homogeneous coordinates for calculations.
            mMeasSensor = mSensors(:,measNumber);

            % Corresponding position in absolute frame. 
            oMeasSensor = oTm * mMeasSensor ;            
            
            % Comparison of the distance to the lines to decide which type
            % of line we keep (horizontal or vertical)
            oMeasSensorNormalized=oMeasSensor./[xSpacing;ySpacing;1];
            oMeasSensorNormalized=mod(oMeasSensorNormalized,[1,1,1]);
            if oMeasSensorNormalized(1)>0.5
                oMeasSensorNormalized(1)=1-oMeasSensorNormalized(1);
            end
            if oMeasSensorNormalized(2) >0.5
                oMeasSensorNormalized(2)=1-oMeasSensorNormalized(2);
            end
            
            if oMeasSensorNormalized(1)<oMeasSensorNormalized(2) % Vertical line
                % Due to measurement and localization errors, the previously
                % calculated position does not match exactly a vertical line.
                % Which actual vertical line is the closest? It will be the
                % candidate line for the update phase of the state estimator.
                oLineX = round(oMeasSensor(1)/xSpacing)*xSpacing ;

                % Measure: x coordinate of the sensor in R0.
                Y = oMeasSensor(1);

                % The expected measurement is the x coordinate of the real 
                % line in the robot frame.
                Yhat = oLineX ;
                
                C = [ 1 0 -mMeasSensor(1)*sin(X(3))-mMeasSensor(2)*cos(X(3))];
                innov = Y - Yhat ;   
                dMaha = abs(innov) * sqrt( 1 / ( C*P*C.' + Qgamma) ) ;
                LogData( t , 'measurement' , X , P , [0;0] , oMeasSensor(1:2) ) ;
                
                % Calculation of the Kalman Filter
                if dMaha <= mahaThreshold & abs(oMeasSensorNormalized(1)-oMeasSensorNormalized(2))>0.1
                    K = 1/( C*P*C.' + Qgamma) * P * C.'  ;
                    X = X + innov.*K ;
                    P = (eye(length(X)) - K*C) * P ;
                    LogData( t , 'update' , X , P , [0;0] , [0;0] ) ;
                end
            
            else % Horizontal line
                
                % Due to measurement and localization errors, the previously
                % calculated position does not match exactly a horizontal line.
                % Which actual horizontal line is the closest? It will be the
                % candidate line for the update phase of the state estimator.
                oLineY = round(oMeasSensor(2)/ySpacing)*ySpacing ;

                % Measure: y coordinate of the sensor in Rm.
                Y = oMeasSensor(2);

                % The expected measurement is the y coordinate of the real 
                % line in the robot frame.
                Yhat = oLineY ;

                
                C = [ 0 1 mMeasSensor(1)*cos(X(3))-mMeasSensor(2)*sin(X(3))];
                innov = Y - Yhat ;   
                dMaha = abs(innov) * sqrt( 1 / ( C*P*C.' + Qgamma) ) ;
                LogData( t , 'measurement' , X , P , [0;0] , oMeasSensor(1:2) ) ;
                
                % Calculation of the Kalman Filter
                if dMaha <= mahaThreshold & abs(oMeasSensorNormalized(1)-oMeasSensorNormalized(2))>0.1 %change le 29/01
                    K = 1/( C*P*C.' + Qgamma) * P * C.'  ;
                    X = X + innov.*K ;
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
     nbSensors U sensorState ...
     Pinit Qgamma sigmaTuning Qbeta Qalpha mahaThreshold 

 
LogData( t , 'termination' , X , P , [0;0] , [0;0] ) ;
close(wbHandle) ;
%close all;
PlotResults;
