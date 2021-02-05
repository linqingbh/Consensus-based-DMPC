function [p_x,p_y,p_z,V_x,V_y,V_z,u_x,u_y,u_z]= formation_consensus_p(p_x,p_y,p_z,V_x,V_y,V_z,u_x,u_y,u_z,dt)
% consensus algorithm step bu step
    k_p=0.01;
    bata=2;
    deta_x=[0 -30 -60 -90 -45 0 45 90 60 30];   % 三角形   
    deta_y=[0 -51.9 -103.8 -155.7 -155.7 -155.7 -155.7 -155.7 -103.8 -51.9];
    deta_z=[0 0 0 0 0 0 0 0 0 0];
    N=10;  
    h_g=120;
    alpha=1;

    A=[0 1 1 1 1 1 1 1 1 1;     % a(ij)
    1 0 1 1 1 1 1 1 1 1;
    1 1 0 1 1 1 1 1 1 1;
    1 1 1 0 1 1 1 1 1 1;
    1 1 1 1 0 1 1 1 1 1;
    1 1 1 1 1 0 1 1 1 1;
    1 1 1 1 1 1 0 1 1 1;
    1 1 1 1 1 1 1 0 1 1;
    1 1 1 1 1 1 1 1 0 1;
    1 1 1 1 1 1 1 1 1 0] ;

    %forward leader's state
    V_x(1)=V_x(1)+dt*u_x(1);
    V_y(1)=V_y(1)+dt*u_y(1);    
          
    p_x(1)=p_x(1)+dt*V_x(1); 
    p_y(1)=p_y(1)+dt*V_y(1);
    p_z(1)=p_z(1)+k_p*(h_g-p_z(1));      
    %compute followers' accleration and forward their states 
    for i=2:N
        for j=2:N   %领航跟随法一致性协议
        u_x(i)=u_x(1)-alpha*(((p_x(i)-deta_x(i))-p_x(1))+bata*(V_x(i)-V_x(1)))-A(i,j)*((p_x(i)-p_x(j))-(deta_x(i)-deta_x(j))+bata*(V_x(i)-V_x(j)));
        u_y(i)=u_y(1)-alpha*(((p_y(i)-deta_y(i))-p_y(1))+bata*(V_y(i)-V_y(1)))-A(i,j)*((p_y(i)-p_y(j))-(deta_y(i)-deta_y(j))+bata*(V_y(i)-V_y(j)));
        u_z(i)=u_z(1)-alpha*(((p_z(i)-deta_z(i))-p_z(1))+bata*(V_z(i)-V_z(1)))-A(i,j)*((p_z(i)-p_z(j))-(deta_z(i)-deta_z(j))+bata*(V_z(i)-V_z(j)));    
        end
    end   
    for i=2:N
       V_x(i)=V_x(i)+dt*u_x(i); %v=vo+at   9followers 
       V_y(i)=V_y(i)+dt*u_y(i);
       V_z(i)=V_z(i)+dt*u_z(i);
       V(i)=sqrt((V_x(i))^2+(V_y(i))^2+(V_z(i))^2);
       theta(i)=atan(V_z(i)/V_x(i))*180/pi;
       psi(i)=atan(V_y(i)/V_x(i))*180/pi;
    end  
    for i=2:N
        p_x(i)=p_x(i)+dt*V_x(i); %x=xo+vt
        p_y(i)=p_y(i)+dt*V_y(i);
        p_z(i)=p_z(i)+dt*V_z(i);           
    end