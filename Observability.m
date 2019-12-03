
syms a1 a2 b1 b2 theta x y v w det det2 det3; 

g1x= x + a1*cos(theta) -b1*sin(theta);
g2x = x + a2*cos(theta) -b2*sin(theta);

g1y = y + a1*sin(theta)+ b1*cos(theta);
g2y = y + a2*sin(theta)+ b2*cos(theta);


%%Fonctions g1x et g2x
%Calcul de la jacobienne g1x
dg1x= jacobian(g1x,[x;y;theta]).';
%Calcul de la jacobienne g2x
dg2x= jacobian(g2x,[x;y;theta]).' ;

%Expression de la fonction f telle que dX/dt=f(X)
f = [v*cos(theta) ; v*sin(theta) ; w];

%Calcul de la dérivée de Lie pour g1 
Lf_g1x = dg1x.'*f;

dLf_g1x = jacobian(Lf_g1x, [x;y;theta]).';

%Calcul de la dérivée de Lie pour g2 
Lf_g2x = dg2x.'*f;

dLf_g2x = jacobian(Lf_g2x, [x;y;theta]).';

%Calcul de la dérivée seconde de Lie pour g1

Lf2_g1x = dLf_g1x.' *f;
dLf2_g1x=jacobian(Lf2_g1x, [x;y;theta]).';


%Calcul de la dérivée seconde de Lie pour g2

Lf2_g2x = dLf_g2x.' *f;
dLf2_g2x=jacobian(Lf2_g2x, [x;y;theta]).';


%Calcul de la dérivée troisième de Lie pour g1
Lf3_g1x = dLf2_g1x.' *f;
dLf3_g1x=jacobian(Lf3_g1x, [x;y;theta]).';

%Calcul de la dérivée troisième de Lie pour g2
Lf3_g2x = dLf2_g2x.' *f;
dLf3_g2x=jacobian(Lf3_g2x, [x;y;theta]).';

%%Fonctions g1y et g2y
%Calcul de la jacobienne g1y
dg1y= jacobian(g1y,[x;y;theta]).';
%Calcul de la jacobienne g2x
dg2y= jacobian(g2y,[x;y;theta]).' ;



%Calcul de la matrice d'observabilité
O_rond1 = [dg1x dg2x dg1y];
O_rond2 = [dg1x dg2x dg2y];
O_rond3 = [dg1x dg2y dLf_g1x];
rang = rank(O_rond1);
det = det(O_rond1)
det2 = det(O_rond2)
det3 = det(O_rond3)









