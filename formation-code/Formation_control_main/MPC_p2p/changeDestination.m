function [Pd] = changeDestination(Pd,p_x,p_y,p_z,t,K)
    
if size(p_x,2)>1
    for i = 1:K
        Pd(3*i-2) =  p_x(t);
        Pd(3*i-1) =  p_y(t);
        Pd(3*i)   =  p_z(t);
        t = t+1;
    end
end

if size(p_x,2) == 1
    for i = 1:K
        Pd(3*i-2) =  p_x;
        Pd(3*i-1) =  p_y;
        Pd(3*i)   =  p_z;
    end
end

    %     
%     for i = 1:K
%         Pd(3*i-2) = p_x(agenti,t);
%         Pd(3*i-1) = p_y(agenti,t);
%         Pd(3*i)   = p_z(agenti,t);
%         t = t+1;
%     end


