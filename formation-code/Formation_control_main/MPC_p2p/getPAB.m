function [PAB] = getPAB(i,phi,A,B)

PAB = [];
c = phi*A^(i-1)*B;
while i>0 
    PAB = [PAB c];
    i = i-1;
    c = phi*A^(i-1)*B;
end


