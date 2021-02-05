function [p_x,p_y,p_z,V_x,V_y,V_z]= formation_consensus()

    N=10;                   % 1 leader + 9 followers
    countmax=1000;
    dt=0.02;
    rd=5;
    SafeRegion =3;
    Kca=150;
    bata=2;
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
    % deta_x=[0 5 10 15 20 25 30 35 40 45];   % 一字型  
    % deta_y=[0 0 0 0 0 0 0 0 0 0];
    % deta_z=[0 0 0 0 0 0 0 0 0 0];
      
    %  deta_x=[0 -30 -60 -90 -45 0 45 90 60 30];   % 三角形   
    %  deta_y=[0 -30 -60 -90 -90 -90 -90 -90 -60 -30];
    %  deta_z=[0 0 0 0 0 0 0 0 0 0];
    
    deta_x=[0 -30 -60 -90 -45 0 45 90 60 30];   % 三角形   
    deta_y=[0 -51.9 -103.8 -155.7 -155.7 -155.7 -155.7 -155.7 -103.8 -51.9];
    deta_z=[0 0 0 0 0 0 0 0 0 0];
     
    h_g=120;
    %% 初始化 位置、速度、加速度
    p_x(:,1)=[-25;-70;-50;-20;-10;10;-10;20;20;10];
    p_y(:,1)=[9;-1.2;4;1;10;1.5;2;8;4;10];
    p_z(:,1)=[150;140;100;160;175;145;180;168;135;165];
    
    % p_x(:,1)=[-25;-70;-50;-20;-10;10;-10;20;20;10];
    % p_y(:,1)=[9;-1.2;4;1;10;1.5;2;8;4;10];
    % p_z(:,1)=[-2.1;-1;1;-1.5;2.5;2;3;11;3;-2];
    
    V_x(:,1)=[20;10;10;10;8;12;13;9;12;10];
    V_y(:,1)=[20;9;11;10;12;9;10;12;10;9];
    V_z(:,1)=[20;12;10;10;10;11;9;10;9;10];
    
    u_x(:,1)=[0;0;0;0;0;0;0;0;0;0];
    u_y(:,1)=[0;0;0;0;0;0;0;0;0;0];
    u_z(:,1)=[0;0;0;0;0;0;0;0;0;0];
    k=0;
    k_p=0.01;
    
    %% 开始循环
    for count=1:countmax
        k=k+1;
    %   u_x(1,k)=0;        %leader 匀速 
    %   u_y(1,k)=0;
    %   u_z(1,k+1)=0;
        V_x(1,k+1)=V_x(1,k)+dt*u_x(1,k);
        V_y(1,k+1)=V_y(1,k)+dt*u_y(1,k);    
    %   u_z(1,k+1)=V_z(1,k)+k_p*(h_g-p_z(1,k));  %二阶高度保持
    %   V_z(1,k+1)=V_z(1,k)+dt*u_z(1,k);        
        p_x(1,k+1)=p_x(1,k)+dt*V_x(1,k); 
        p_y(1,k+1)=p_y(1,k)+dt*V_y(1,k);
    %   p_z(1,k+1)=p_z(1,k)+dt*V_z(1,k);
        p_z(1,k+1)=p_z(1,k)+k_p*(h_g-p_z(1,k));      
    %   theta(1,k+1)=atan(V_z(1,k+1)/V_x(1,k+1))*180/pi; 
    %   psi(1,k+1)=atan(V_y(1,k+1)/V_x(1,k+1))*180/pi;
    %   V(1,k+1)=sqrt((V_y(1,k+1))^2+(V_y(1,k+1))^2+(V_z(1,k+1))^2);
        for i=2:N
            for j=2:N   %领航跟随法一致性协议
            u_x(i,k+1)=u_x(1,k)-alpha*(((p_x(i,k)-deta_x(i))-p_x(1,k))+bata*(V_x(i,k)-V_x(1,k)))-A(i,j)*((p_x(i,k)-p_x(j,k))-(deta_x(i)-deta_x(j))+bata*(V_x(i,k)-V_x(j,k)));
            u_y(i,k+1)=u_y(1,k)-alpha*(((p_y(i,k)-deta_y(i))-p_y(1,k))+bata*(V_y(i,k)-V_y(1,k)))-A(i,j)*((p_y(i,k)-p_y(j,k))-(deta_y(i)-deta_y(j))+bata*(V_y(i,k)-V_y(j,k)));
            u_z(i,k+1)=u_z(1,k)-alpha*(((p_z(i,k)-deta_z(i))-p_z(1,k))+bata*(V_z(i,k)-V_z(1,k)))-A(i,j)*((p_z(i,k)-p_z(j,k))-(deta_z(i)-deta_z(j))+bata*(V_z(i,k)-V_z(j,k)));    
    %       h_ddot(i,k+1)=h_ddot(1,k)-alpha*(((h(i,k)-deta_z(i))-h(1,k))+bata*(h_dot(i,k)-h_dot(1,k)))-A(i,j)*((h(i,k)-h(j,k))-(deta_z(i)-deta_z(j))); 
            end
        end   
        for i=2:N
           V_x(i,k+1)=V_x(i,k)+dt*u_x(i,k); %v=vo+at   9followers 
           V_y(i,k+1)=V_y(i,k)+dt*u_y(i,k);
           V_z(i,k+1)=V_z(i,k)+dt*u_z(i,k);
           V(i,k+1)=sqrt((V_x(i,k+1))^2+(V_y(i,k+1))^2+(V_z(i,k+1))^2);
           theta(i,k+1)=atan(V_z(i,k+1)/V_x(i,k+1))*180/pi;
           psi(i,k+1)=atan(V_y(i,k+1)/V_x(i,k+1))*180/pi;
        end  
        for i=2:N
            p_x(i,k+1)=p_x(i,k)+dt*V_x(i,k); %x=xo+vt
            p_y(i,k+1)=p_y(i,k)+dt*V_y(i,k);
            p_z(i,k+1)=p_z(i,k)+dt*V_z(i,k);
    %       p_z(i,k+1)=120;            
        end
    end
    



