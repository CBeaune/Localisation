
syms a1 a2 b1 b2 theta x y v w; 

g1= x + a1*cos(theta) -b1*sin(theta);
g2 = x + a2*cos(theta) -b2*sin(theta);

%Calcul de la jacobienne pour xm=0,ym=0
dg1= jacobian(g1,[x;y;theta]).';
%Calcul de la jacobienne pour xm=0, ym=1
dg2= jacobian(g2,[x;y;theta]).' ;

%Expression de la fonction f telle que dX/dt=f(X)
f = [v*cos(theta) ; v*sin(theta) ; w];

%Calcul de la dérivée de Lie pour g1 
Lf_g1 = dg1.'*f;

dLf_g1 = jacobian(Lf_g1, [x;y;theta]).';

%Calcul de la dérivée de Lie pour g2 
Lf_g2 = dg2.'*f;

dLf_g2 = jacobian(Lf_g2, [x;y;theta]).';

%Calcul de la dérivée seconde de Lie pour g1

Lf2_g1 = dLf_g1.' *f;
dLf2_g1=jacobian(Lf2_g1, [x;y;theta]).';


%Calcul de la dérivée seconde de Lie pour g2

Lf2_g2 = dLf_g2.' *f;
dLf2_g2=jacobian(Lf2_g2, [x;y;theta]).';


%Calcul de la dérivée troisième de Lie pour g1
Lf3_g1 = dLf2_g1.' *f;
dLf3_g1=jacobian(Lf3_g1, [x;y;theta]).';

%Calcul de la dérivée troisième de Lie pour g2
Lf3_g2 = dLf2_g2.' *f;
dLf3_g2=jacobian(Lf3_g2, [x;y;theta]).';



%Calcul de différentes matrices d'observabilité
O_rond1 = [dg1 dg2 dLf_g1 dLf_g2 dLf2_g1 dLf2_g2 dLf3_g1 dLf3_g2]

    %un seul capteur fonctionne
O_rond2 = [dg1  dLf_g1 dLf2_g1 dLf3_g1];
    %en ligne droite

O_rond3 = [dg1 dg2 dLf_g1 dLf_g2 dLf2_g1 dLf2_g2 dLf3_g1 dLf3_g2]
    %on ne croise que des x=cstes
O_rond4 = [dg2 dLf_g1 dLf2_g2];

%Calcul du déterminant 
rank1=rank(O_rond1)
rank2=rank(O_rond2)
rank3=rank(O_rond3)
% det4=simplify(det(O_rond4))



%%%   On a perte d'observabilité pour w=0, ce qui correspond aux
%%%   trajectoires en ligne droite 



