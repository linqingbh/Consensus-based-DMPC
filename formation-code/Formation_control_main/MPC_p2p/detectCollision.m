function [D,kci] = detectCollision(P,P1,K,r_min)

kci = 1;
D = 0;
for kci = 1:K
    if(  norm(P(3*kci-2:3*kci)-P1(3*kci-2:3*kci))<r_min     )
        D = 1; 
        break;
    elseif kci == K
        D = 0;
        break;
    end
end



  

