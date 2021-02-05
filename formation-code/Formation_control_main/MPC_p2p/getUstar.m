function [Ustar] = getUstar(ui,K)

Ustar = [ui'];
c = zeros(3,1);

for i = 1:K-1
Ustar = [Ustar c'];
end
Ustar = Ustar';


