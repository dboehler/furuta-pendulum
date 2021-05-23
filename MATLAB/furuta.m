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

C = [0 0 1 0];
D = [0];

sys = ss(A,B,C,D);

p1 = -16+5j;
p2 = -16-5j;
p3 = -.2*16.7417;
p4 = -160.7417;

K = place(A,B,[p1 p2 p3 p4]);
sys_cl = ss(A-B*K,B,C,0);
tf = ss2tf((A-B*K),B,C,D)

t = 0:0.01:2;
u = zeros(size(t));
x0 = [0.1 0.1 0 0];
[y,t,x] = lsim(sys_cl,u,t,x0);
plot(t,y,t,x(4))
%display poles
eig(A)