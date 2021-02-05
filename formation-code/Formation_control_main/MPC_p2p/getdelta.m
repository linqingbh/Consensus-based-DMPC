function [delta] = getdelta(K)

delta = eye(3*K);

for i = 1:K-1
delta(3*i+1:(i+1)*3 ,3*(i-1)+1:3*i) = -1*eye(3);
end
