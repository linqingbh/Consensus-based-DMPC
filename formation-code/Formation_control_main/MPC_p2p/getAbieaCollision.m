function [Aieq,bieq] = getAbieaCollision(P0,P1,kci,r_min,K,lambda,A0,X0,epsilon)
 
     Aieq = [-1*eye(3*K) ; eye(3*K)];
     bieq = ones(6*K,1);
    % Aieq = [];
     %bieq = [];
     %lambda_v = getLambda_v(K);
%     v_ij = P0(3*kci-2:3*kci)-P1(3*kci-2:3*kci);
%     xi_ij = norm( v_ij);
%     
%     mu_ij = [zeros(3*(kci-1),1)'  v_ij' zeros(3*(K-kci),1)'  ]';
%     rho = r_min*xi_ij + xi_ij^2 + v_ij'*P0(3*kci-2:3*kci);
%    
%     A = mu_ij'*lambda;
%     b = rho - mu_ij'*A0*X0 + epsilon* xi_ij;
%    
%     Aieq = [Aieq ; -A; ];
%     bieq = [bieq ; -b; ];
