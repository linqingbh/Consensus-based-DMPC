function [P1] = changeMatrix(P)

k1 = size(P,1);
P1 = [];
for i = 1:3:k1
    P1 = [ P1;P(i) P(i+1) P(i+2) ];
end
