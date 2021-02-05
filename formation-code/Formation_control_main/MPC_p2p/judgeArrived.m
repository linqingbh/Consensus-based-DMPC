function [J] = judgeArrived(Pd,Pd1,X0,X1)

if norm(Pd(1:3)-X0(1:3))<=0.3 || norm(Pd1(1:3)-X1(1:3))<=0.3
    J = 1;
else
    J = 0;
end


