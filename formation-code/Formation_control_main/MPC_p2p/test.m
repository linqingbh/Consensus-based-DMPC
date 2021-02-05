% nci = 5;
% K = 10;
% epsilon = -3;
% U = [ones(30,1)*2];
% E = [ones(nci,1)*epsilon];
% H_eps = [zeros(3*K,3*K) zeros(3*K,nci);zeros(nci,3*K) eye(nci)];
% f_eps = [zeros(3*K,1)' ones(nci,1)']';
% U_aug = [U;E];
% 
% E1 = U_aug'*H_eps*U_aug-f_eps'*U_aug;
% E2 = E'*E-epsilon*nci;
fps = 40;
myVideo = VideoWriter('test2.avi'); 
myVideo.FrameRate = fps; 
open(myVideo); 
figure
for k = 1:size(Xplot(1,:),2)
    plot3(Xplot(1,k),Xplot(1,k),Xplot(1,k),'r.'),hold on
    plot3(Xplot1(1,k),Xplot1(1,k),Xplot1(1,k),'b.');
    axis([0,max(Xplot(1,:)),0,max(Xplot(2,:)),0,max(Xplot(3,:))]),drawnow
    frame = getframe(gcf);
    im = frame2im(frame); 
    writeVideo(myVideo,im); 
end
close(myVideo);