function [XallStates,Xfw_States] = fixedwing(XallStates,h,Xfw_States)
   % states now 
    desired_pos = XallStates;
    x_now = Xfw_States(1);
    y_now = Xfw_States(2);
    z_now = Xfw_States(3);
    v_now = Xfw_States(4);
    theta_now = Xfw_States(5);
    phi_now = Xfw_States(6);

    %dynamic limits
    v_min = 35; 
    v_max = 45;
    theta_min = -pi/6;
    theta_max =  pi/6;
    a_max = 1;
    a_min = -1;
    w_theta_max = pi/36;
    w_theta_min = -pi/36;
    w_phi_max = pi/36;
    w_phi_min = -pi/36;
    error_mer = 10000000000000;
    w_theta_part =  pi/360;
    w_phi_part = pi/360;


    a_record = 0;
    w1_record = 0;
    w2_record = 0;

   

    for a = a_min : 0.01:a_max
        for w1 = w_theta_min: w_theta_part :w_theta_max
            for w2 = w_phi_min: w_phi_part :w_phi_max
                v = v_now;
                theta = theta_now;
                phi = phi_now;
                x = x_now;
                y = y_now;
                z = z_now;

                v = v + a*h;
                theta = theta + w1*h;
                phi = phi + w2*h;
                if(abs(theta)<= theta_max && v_min<=v && v<=v_max)
                %if(abs(theta)<= theta_max)    
                    x = x + h*v*cos(theta)*cos(phi);
                    y = y + h*v*cos(theta)*sin(phi);
                    z = z + h*v*sin(theta);
                else
                    continue;
                end

                error =  norm([x-desired_pos(1) ;y-desired_pos(2) ;z-desired_pos(3)] );
                if(error<=error_mer) %record the least error and all inputs
                    error_mer = error;
                    a_record = a;
                    w1_record = w1;
                    w2_record = w2;
                end

            end
        end
    end



    Xfw_States(4) = v_now + a_record*h;
    Xfw_States(5) = theta_now + w1_record*h;
    Xfw_States(6) = phi_now + w2_record*h;
    Xfw_States(1) = x_now +  h*Xfw_States(4) *cos( Xfw_States(5))*cos(Xfw_States(6));
    Xfw_States(2) = y_now +  h*Xfw_States(4) *cos( Xfw_States(5))*sin(Xfw_States(6));
    Xfw_States(3) = z_now +  h*Xfw_States(4) *sin( Xfw_States(5));
    
    XallStates(1) = Xfw_States(1);
    XallStates(2) = Xfw_States(2);
    XallStates(3) = Xfw_States(3);
    XallStates(4) = Xfw_States(4)*cos(Xfw_States(5))*cos(Xfw_States(6));
    XallStates(5) = Xfw_States(4)*cos(Xfw_States(5))*sin(Xfw_States(6));
    XallStates(6) = Xfw_States(4)*sin(Xfw_States(5));

    

