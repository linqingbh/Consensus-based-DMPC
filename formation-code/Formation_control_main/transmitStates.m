function [p_x,p_y,p_z,V_x,V_y,V_z] = transmitStates(XallStates,p_x,p_y,p_z,V_x,V_y,V_z,N)

    for i = 1:N

        p_x(i) = XallStates(1,i);
        p_y(i) = XallStates(2,i);
        p_z(i) = XallStates(3,i);
        V_x(i) = XallStates(4,i);
        V_y(i) = XallStates(5,i);
        V_z(i) = XallStates(6,i);
    end
