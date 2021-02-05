close all;clc;
addpath('MPC_p2p')
load('matlab1.mat') % leader trajectories

N = 5;
K = 15;
h = 0.2; %time step
dt = 0.2;
U = zeros(3*K,1); % ax0,ay0,az0,......,ax9,ay9,az9
x = zeros(6,1);

A = eye(6);
A(1:3,4:6) = h*eye(3);
B = zeros(6,3);
B(1:3,1:3) = 0.5*(h^2)*eye(3);
B(4:6,1:3) = h*eye(3);

phi = zeros(3,6);
phi(1:3,1:3) = eye(3);
lambda = getLambda(K , phi,A,B);
delta = getdelta(K);
lambda_v = getLambda_v(K);
%X0 =  zeros(6,1); %initial state
%X0 = [0;0;0;-1;-1;-3];

A0 = getA0(K ,phi,A);
epsilon = 0.05;
r_min = 3;

%weight matrix
Q = eye(3*K);
%Q(1:3*K-3,1:3*K-3) = zeros(3*K-3,3*K-3); % whether more aggressive
%Q(1:3*K-6,1:3*K-6) = zeros(3*K-6,3*K-6);
R = eye(3*K)*0.01;
S = eye(3*K)*0.1;

% U constraints acceleration
Aieq = [-1*eye(3*K);eye(3*K)];
bieq = ones(6*K,1)*3;
%timesteps = 1000;
timesteps = 500;

%-----------all agent states----------
agentSum = N;
ui_all = zeros(3,N); % all initial input
XallStates = zeros(6,N);
Xfw_States = zeros(6,N); %fixed wing states x y z v theta phi
Pd_all = ones(3*K,N);
X1 = [];
X2 =[];
Xplot = [];
Xplot2 = [];
bata=2;
alpha=1;

Adj=[0 1 1 1 1 1 1 1 1 1;     % a(ij)
   1 0 1 1 1 1 1 1 1 1;
   1 1 0 1 1 1 1 1 1 1;
   1 1 1 0 1 1 1 1 1 1;
   1 1 1 1 0 1 1 1 1 1;
   1 1 1 1 1 0 1 1 1 1;
   1 1 1 1 1 1 0 1 1 1;
   1 1 1 1 1 1 1 0 1 1;
   1 1 1 1 1 1 1 1 0 1;
   1 1 1 1 1 1 1 1 1 0] ;
%----------------------------------

% agent 1
% for i = 1:N 
% XallStates(:,i) = [p_x(i,1),p_y(i,1),p_z(i,1),V_x(i,1),V_y(i,1),V_z(i,1)];
% XallStates(:,i) = [p_x(i,1),p_y(i,1),p_z(i,1),0,0,0];
% 
% end

Xfw_States(:,1) = [800 ,300,120, 40,0,0];
Xfw_States(:,2) = [600 ,200,120 ,40,0,0];
Xfw_States(:,3) = [400 ,500,120 ,40,0,0];
Xfw_States(:,4) = [200 ,1000,120 ,40,0,0];
Xfw_States(:,5) = [ 0 ,   0,120 ,40,0,0];

for i = 1:N
    XallStates(1,i) = Xfw_States(1,i);
    XallStates(2,i) = Xfw_States(2,i);
    XallStates(3,i) = Xfw_States(3,i);
    XallStates(4,i) = Xfw_States(4,i)*cos(Xfw_States(5,i))*cos(Xfw_States(6,i));
    XallStates(5,i) = Xfw_States(4,i)*cos(Xfw_States(5,i))*sin(Xfw_States(6,i));
    XallStates(6,i) = Xfw_States(4,i)*sin(Xfw_States(5,i));
end
% XallStates(:,1) = [p_xr(1,1),p_yr(1,1),p_zr(1,1),0,0,0];
% XallStates(:,2) = [260 ,200,125 ,0,0,0];
% XallStates(:,3) = [200 ,210,125 ,0,0,0];
% XallStates(:,4) = [180 ,180,125 ,0,0,0];
% XallStates(:,5) = [140 ,180,125 ,0,0,0];



p_x=[0;0;0;0;0;0;0;0;0;0];
p_y=[0;0;0;0;0;0;0;0;0;0];
p_z=[0;0;0;0;0;0;0;0;0;0];

V_x=[0;0;0;0;0;0;0;0;0;0];
V_y=[0;0;0;0;0;0;0;0;0;0];
V_z=[0;0;0;0;0;0;0;0;0;0];


u_x = [0;0;0;0;0;0;0;0;0;0];
u_y = [0;0;0;0;0;0;0;0;0;0];
u_z = [0;0;0;0;0;0;0;0;0;0];

deta_x=[0 60 -120 -180 60 0 0 0 0 0];   
deta_y=[0 0 0 0 0 0 0 0 0 0];
deta_z=[0 0 0 0 0 0 0 0 0 0];

% deta_x=[0 -100 -200 -300 -400 0 45 90 60 30];   
% deta_y=[0 -50 -100 -150 -200  -155.7 -155.7 -155.7 -103.8 -51.9];
% deta_z=[0 0 0 0 0 0 0 0 0 0];


figure(1)

% ---------------main loop-----------------
for t = 1:timesteps  
  
    %for agenti=1:1
    agenti = 1;
    [Pd_all(:,agenti)] = changeDestination(Pd_all(:,agenti),p_xr(1,1)+10000 ,p_yr(1,1)+10000,p_zr(1,1),t,K);
    u = agentQP(ui_all(:,agenti),K,lambda,Q,delta,S,R,Pd_all(:,agenti),A0,XallStates(:,agenti),Aieq,bieq);
    ui1 = u(1:3); % MPC:using the first optimization input 
%     ui1 = zeros(3,1);
    XallStates(:,agenti) = forwardState(XallStates(:,agenti),ui1,A,B);

   [p_x,p_y,p_z,V_x,V_y,V_z] = transmitStates(XallStates,p_x,p_y,p_z,V_x,V_y,V_z,N);
    % ------consensus----------

    for t1 = 1:K
    for i=2:N
        for j=2:N   
        u_x(i)=u_x(1)-alpha*(((p_x(i)-deta_x(i))-p_x(1))+bata*(V_x(i)-V_x(1)))-Adj(i,j)*((p_x(i)-p_x(j))-(deta_x(i)-deta_x(j))+bata*(V_x(i)-V_x(j)));
        u_y(i)=u_y(1)-alpha*(((p_y(i)-deta_y(i))-p_y(1))+bata*(V_y(i)-V_y(1)))-Adj(i,j)*((p_y(i)-p_y(j))-(deta_y(i)-deta_y(j))+bata*(V_y(i)-V_y(j)));
        u_z(i)=u_z(1)-alpha*(((p_z(i)-deta_z(i))-p_z(1))+bata*(V_z(i)-V_z(1)))-Adj(i,j)*((p_z(i)-p_z(j))-(deta_z(i)-deta_z(j))+bata*(V_z(i)-V_z(j)));     
        end  
    end   
    
    for i=2:N
        V_x(i)=V_x(i)+dt*u_x(i); %v=vo+at   9followers 
        V_y(i)=V_y(i)+dt*u_y(i);
        V_z(i)=V_z(i)+dt*u_z(i);
     end  
     for i=2:N
         p_x(i)=p_x(i)+dt*V_x(i); %x=xo+vt
         p_y(i)=p_y(i)+dt*V_y(i);
         p_z(i)=p_z(i)+dt*V_z(i);     
     end
    end

     for agenti=2:N
        [Pd_all(:,agenti)] = changeDestination(Pd_all(:,agenti),p_x(agenti),p_y(agenti),p_z(agenti),t,K);
        u = agentQP(ui_all(:,agenti),K,lambda,Q,delta,S,R,Pd_all(:,agenti),A0,XallStates(:,agenti),Aieq,bieq);
        ui1 = u(1:3); % MPC:using the first optimization input 
        XallStates(:,agenti) = forwardState(XallStates(:,agenti),ui1,A,B);
     end

     % fixedwing autopilot
     for agenti = 1:N
     [XallStates(:,agenti),Xfw_States(:,agenti)] = fixedwing(XallStates(:,agenti),h,Xfw_States(:,agenti));
         
     end
     
        u_x = [0;0;0;0;0;0;0;0;0;0];
        u_y = [0;0;0;0;0;0;0;0;0;0];
        u_z = [0;0;0;0;0;0;0;0;0;0];


        plot3(p_xr(1,:),p_yr(1,:),p_zr(1,:)) ;  hold on;drawnow
        scatter3(XallStates(1,1) , XallStates(2,1) , XallStates(3,1),'g.'); hold on;drawnow
        scatter3(XallStates(1,2) , XallStates(2,2) , XallStates(3,2),'r^'); hold on;drawnow
        scatter3(XallStates(1,3) , XallStates(2,3) , XallStates(3,3),'b^'); hold on;drawnow
        scatter3(XallStates(1,4) , XallStates(2,4) , XallStates(3,4),'k^'); hold on;drawnow
        scatter3(XallStates(1,5) , XallStates(2,5) , XallStates(3,5),'m^'); hold on;drawnow
        axis equal;
%         scatter3(XallStates(1,6) , XallStates(2,6) , XallStates(3,6),'g^'); hold on;drawnow
%         scatter3(XallStates(1,7) , XallStates(2,7) , XallStates(3,7),'r^'); hold on;drawnow
%         scatter3(XallStates(1,8) , XallStates(2,8) , XallStates(3,8),'b^'); hold on;drawnow
%         scatter3(XallStates(1,9) , XallStates(2,9) , XallStates(3,9),'k^'); hold on;drawnow
%         scatter3(XallStates(1,10) , XallStates(2,10) , XallStates(3,10),'g^'); hold on;drawnow

     
    for i=1:N 
        X1 = [X1 XallStates(:,i)' ];
       
    end
    Xplot = [Xplot;X1];
    X1 =[];
    
     for i=1:N 
        X2 = [X2 Xfw_States(:,i)' ];
        
    end
    Xplot2 = [Xplot2;X2];
    X2 =[];
    if(size(Xplot2,1)==400) % 400 could change as you wish
        break;
    end

end % end for timestep

save('5jia.mat','Xplot2')
% After this run file 5jia.mat is updated if change some initial condition
%And what you need to do is copy the file to the folder 'data'
