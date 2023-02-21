function vEE = get_vEE(x)


theta1  = x(1);
theta2  = x(2);

psi1    = x(3);
psi2    = x(4);
q1      = x(5);
q2      = x(6);

dpsi1    = x(7);
dpsi2    = x(8);
dq1      = x(9);
dq2      = x(10);

t2 = q1+q2;
t3 = cos(t2);
t4 = sin(t2);
vEE = [dq2.*t4.*(-1.7e+1./5.0e+1)-dq1.*(t4.*(1.7e+1./5.0e+1)+sin(q1).*(1.7e+1./5.0e+1));dq1.*(t3.*(1.7e+1./5.0e+1)+cos(q1).*(1.7e+1./5.0e+1))+dq2.*t3.*(1.7e+1./5.0e+1);0.0];
