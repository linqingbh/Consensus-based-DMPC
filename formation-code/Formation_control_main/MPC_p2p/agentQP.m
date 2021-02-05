function [u] = agentQP(ui,K,lambda,Q,delta,S,R,Pd,A0,X0,Aieq,bieq)

Ustar = getUstar(ui,K); 

%---Quadratic Programming

H = (lambda'*Q*lambda)+(delta'*S*delta)+R;
f = - (Pd'*Q*lambda-(A0*X0)'*Q*lambda)-(Ustar'*S*delta);

u = quadprog(H,f,Aieq,bieq);
