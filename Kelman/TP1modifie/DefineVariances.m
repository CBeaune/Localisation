% Set the parameters which have the "Determined by student" comment to tune
% the Kalman filter. Do not modify anything else in this file.

% Uncertainty on initial position of the robot.

sigmaX     = 3;         % Determined by student 
sigmaY     = 3;         % Determined by student 
sigmaTheta = 2*pi/180 ;   % Determined by student 
Pinit = diag( [sigmaX^2 sigmaY^2 sigmaTheta^2] ) ;


% Measurement noise.

sigmaXmeasurement = 18/sqrt(12) ;  % Determined by student 
sigmaYmeasurement = 10/sqrt(12) ;  % Determined by student 
Qgamma = diag( [sigmaXmeasurement^2 sigmaYmeasurement^2] ) ;


% Input noise

sigmaTuning = 0.1 ; 
Qwheels = sigmaTuning^2 * eye(2) ;
Qbeta   = jointToCartesian * Qwheels * jointToCartesian.' ; 

% State noise
 
Qalpha = zeros(3) ;

% Mahalanobis distance threshold

mahaThreshold = sqrt(chi2inv(0.95,2));  % Determined by student
