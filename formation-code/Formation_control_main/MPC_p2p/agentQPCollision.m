function [u] = agentQPCollision(ui,K,lambda,Q,delta,S,R,Pd,A0,X0,P,P1,kci,epsilon,r_min)

    Ustar = getUstar(ui,K); 

    %---Quadratic Programming
    C = zeros(3,3*K);
    C(1,3*kci-2) = 1; C(2,3*kci-1) = 1; C(3,3*kci) = 1;
    
    Perror = P(kci:kci+2)-P1(kci:kci+2); 
    k1 = 1000000001000/(norm(Perror-r_min));
    H = (lambda'*Q*lambda)+(delta'*S*delta)+R +k1*(lambda'*lambda);
    f = - (Pd'*Q*lambda-(A0*X0)'*Q*lambda)-(Ustar'*S*delta) ...
    +k1*(X0'*A0'*C'*C*lambda-P1(kci:kci+2)'*C*lambda);
    % + k2*0.5*C*lambda
    [Aieq,bieq] = getAbieaCollision(P,P1,kci,r_min,K,lambda,A0,X0,epsilon);
    
    u = quadprog(H,f,Aieq,bieq);
    

