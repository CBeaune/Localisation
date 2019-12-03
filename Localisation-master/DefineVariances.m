% Set the parameters which have the "Determined by student" comment to tune
% the Kalman filter. Do not modify anything else in this file.

% Uncertainty on initial position of the robot.

sigmaX     = 0;         % Determined by student 
sigmaY     = 0;         % Determined by student 
sigmaTheta = 0*pi/180 ;   % Determined by student 
Pinit = diag( [sigmaX^2 sigmaY^2 sigmaTheta^2] ) ;


% Measurement noise.

%Constantes nécessaires au calcul
T = 1/200 ; %Periode d echantillonage des capteurs
Vmax = 2 ; %Vitesse maximale admissible par le robot. A verifier. Pour l'instant on l'estime a 2m/s

sigmaXmeasurement = 1/sqrt(3)*(width/2+T*Vmax) ;  % Determined by student 
sigmaYmeasurement = 1/sqrt(3)*(width/2+T*Vmax) ;
%10/sqrt(12) ;  % Determined by student 
Qgamma = diag( [sigmaXmeasurement^2 sigmaYmeasurement^2] ) ;


% Input noise

sigmaTuning = 0.5 ; 
Qwheels = sigmaTuning^2 * eye(2) ;
Qbeta   = jointToCartesian * Qwheels * jointToCartesian.' ; 

% State noise
 
Qalpha = zeros(3) ;

% Mahalanobis distance threshold

mahaThreshold = sqrt(chi2inv(0.95,2));  % Determined by student
