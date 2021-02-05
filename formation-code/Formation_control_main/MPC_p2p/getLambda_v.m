function [lambda_v] = getLambda_v(K)

    lambda_v = zeros(3*K , 3*K);
    
    for i = 1:K
        C = [];
        t = i;
        while t>0 
            C = [C eye(3)];
            t = t-1;           
        end
        lambda_v( 3*(i-1)+1:3*i, 1:3*i) = C ;
        
    end
    
    