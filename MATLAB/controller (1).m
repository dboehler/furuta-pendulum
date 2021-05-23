function T = controller(theta0, dtheta0, theta1, dtheta1)
L0 = 0.12;
L1 = 0.12;
g = 9.81;
m0 = 0.10;
m1 = 0.10;
Lmass = 0.5*L1;
I = (1/3)*m0*L0^2;
J = (1/12)*m1*L1^2;
A = [0 1                           0                         0;
     0 0    -g*m1^2*Lmass^2*L0/(I*(J+m1*Lmass^2)+J*m1*L0^2)  0;
     0 0                           0                         1;
     0 0 (I+m1*L0^2)*m1*Lmass*g/(I*(J+m1*Lmass^2)+J*m1*L0^2) 0];
B = [                      0;
    (J+m1*Lmass^2)/(I*(J+m1*Lmass^2)+J*m1*L0^2);
                           0;
    -(m1*Lmass*L0)/(I*(J+m1*Lmass^2)+J*m1*L0^2)];

p1 = -16+1j;
p2 = -16-1j;
p3 = -16.7417;
p4 = -160.7417;

K = place(A,B,[p1 p2 p3 p4]);
states = [theta0, dtheta0, theta1, dtheta1];
T = K*states';



