function [x] = forwardState(x,ui,A,B)

x = A*x+B*ui;
