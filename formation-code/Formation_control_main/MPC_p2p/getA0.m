function [A0] = getA0(K ,phi,A)

    A0 = [];
for i = 1:K
    c = (phi*A^(i))';
    A0 = [A0 c];
end
A0 = A0';

