close all;clc;
K = 15;
h = 0.2;
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

% agent 0
ui = zeros(3,1); %initial input
X0 = [0;0;0;0;0;0]; %initial state
Pd = ones(3*K,1)*50; %destination
%Pd = zeros(3*K,1);
%Pd(4:3*K) = ones(27,1)*10; 

%agent 1
ui1 = zeros(3,1);
X1 = [50;50;50;0;0;0];
Pd1 = zeros(3*K,1)*10;
%Pd1(3*K-2:3*K) = [0;0;0];

%weight matrix
Q = eye(3*K);
%Q(1:3*K-3,1:3*K-3) = zeros(3*K-3,3*K-3); % whether more aggressive
%Q(1:3*K-6,1:3*K-6) = zeros(3*K-6,3*K-6);
R = eye(3*K);
S = eye(3*K);

% U constraints
Aieq = [-1*eye(3*K);eye(3*K)];
bieq = ones(6*K,1);


% 0:0.2:8  40 step
%Jei = U'*(lambda'*Q*lambda)*U - 2*(Pd'*Q*lambda-(A0*X0)'*Q*lambda)*U;

%Jui = U'*R*U;

%Jdelta = U'*(delta'*S*delta)*U-2*(Ustar*S*delta)*U;

%J = Jei+Jui+Jdelta;

Xplot = [X0(1:3)];
Xplot1 = [X1(1:3)];
D = 0;
timesteps = 200;
Uplot = [];
Uplot1 = [];




figure

% ---------------main loop-----------------
for t = 1:timesteps
  
  %if D == 1
   %  u = agentQPCollision(ui,K,lambda,Q,delta,S,R,Pd,A0,X0,P,P1,kci,epsilon,r_min);
    
    %u1 = agentQPCollision(ui1,K,lambda,Q,delta,S,R,Pd1,A0,X1,P1,P,kci,epsilon,r_min);

  %elseif D == 0

    u = agentQP(ui,K,lambda,Q,delta,S,R,Pd,A0,X0,Aieq,bieq);
    
    u1 = agentQP(ui1,K,lambda,Q,delta,S,R,Pd1,A0,X1,Aieq,bieq);
   
%  end

%if ~isempty(u) ~= 0
    ui = u(1:3); % MPC:using the first optimization input 
%end
X0 = forwardState(X0,ui,A,B);
Xplot = [Xplot X0(1:3)];
Uplot = [Uplot ;ui];

%if ~isempty(u1) ~= 0
  ui1 = u1(1:3); 
%end

X1 = forwardState(X1,ui1,A,B);
Xplot1 = [Xplot1 X1(1:3)];
Uplot1 = [Uplot1 ;ui1];

P = A0*X0 + lambda*u;
P1 = A0*X1 + lambda*u1;


[D,kci] = detectCollision(P,P1,K,r_min);


J = judgeArrived(Pd,Pd1,X0,X1);
if J==1
  break;
end

Pplot = reshape(P,[K,3]);

scatter3(X0(1),X0(2),X0(3),'g^'); hold on;drawnow
scatter3(X1(1),X1(2),X1(3),'r^'); hold on;drawnow

% Pplot = changeMatrix(P);
% P1plot = changeMatrix(P1);
% plot3(Pplot(:,1),Pplot(:,2),Pplot(:,3),'b' )
% plot3(P1plot(:,1),P1plot(:,2),P1plot(:,3),'m' )

end % end for
%----------------end loop----------------
 


figure(2) 
plot(Uplot(:,1)); 
% figure(3) 
% plot(Uplot(:,2));
% figure(4) 
% plot(Uplot(:,3)); 
%plot3(Xplot(1,:) , Xplot(2,:) , Xplot(3,:),'r.'); hold on;
%plot3(Xplot1(1,:) , Xplot1(2,:) , Xplot1(3,:),'g');
% 
% fps = 40;
% myVideo = VideoWriter('test2.avi'); 
% myVideo.FrameRate = fps; 
% open(myVideo); 
% figure
% for k = 1:size(Xplot(1,:),2)
%     plot3(Xplot(1,k),Xplot(1,k),Xplot(1,k),'r.'),hold on
%     plot3(Xplot1(1,k),Xplot1(1,k),Xplot1(1,k),'b.');
%     axis([0,max(Xplot(1,:)),0,max(Xplot(2,:)),0,max(Xplot(3,:))]),drawnow
%     frame = getframe(gcf);
%     im = frame2im(frame); 
%     writeVideo(myVideo,im); 
%     
    
    
    
%     
% end
% close(myVideo); 