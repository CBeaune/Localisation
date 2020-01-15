% Set the parameters which have the "Determined by student" comment to tune
% the Kalman filter. Do not modify anything else in this file.

% Uncertainty on initial position of the robot.

sigmaX     = 0.1;         % Determined by student 
sigmaY     = 0.1;         % Determined by student 
sigmaTheta = 0.1*pi/180 ;   % Determined by student 
Pinit = diag( [sigmaX^2 sigmaY^2 sigmaTheta^2] ) ;


% Measurement noise.

%Constantes nÃ©cessaires au calcul
T = samplingPeriod ; %Periode d echantillonage des capteurs
Vmax = topRobotSpeed ; %Vitesse maximale admissible par le robot. A verifier. Pour l'instant on l'estime a 2m/s

sigmaXmeasurement = 1/sqrt(3)*(width/2+T*Vmax) ; %determine par nous ; Vmax*T ; % demandé par le prof
sigmaYmeasurement = 1/sqrt(3)*(width/2+T*Vmax) ; %determine par nous ; Vmax*T ; % demandé par le prof
QgammaX = sigmaXmeasurement^2;
QgammaY = sigmaYmeasurement^2;
Qgamma=diag([QgammaX,QgammaY]);

% Input noise

sigmaTuning = 0.001 ; %0.4 a la base, 0.1 mieux, 0.01 encore mieux, %0.0001 bon %0.003 d'apres charlotte
Qwheels = sigmaTuning^2 * eye(2) ;
Qbeta   = jointToCartesian * Qwheels * jointToCartesian.' ; 

% State noise

Qalpha = zeros(3) ;

% Mahalanobis distance threshold

mahaThreshold = sqrt(chi2inv(0.95,1)); % Determined by student
