function [lambda] = getLambda(K ,phi,A,B)


lambda = zeros(3*K , 3*K);

for i = 1:K
    lambda( 3*(i-1)+1:3*i, 1:3*i) = getPAB(i,phi,A,B) ;
end



