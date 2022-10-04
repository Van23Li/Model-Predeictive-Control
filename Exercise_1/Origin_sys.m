%%%%%%%%%%%%%%%%%%%%% plot origin system %%%%%%%%%%%%%%%%%%%%%
clear all
close all
% A = [4/3, -2/3; 1, 0];
% B = [1; 0];
A = [1.988,-0.998;1,0];
B = [0.125; 0];
C = [-2/3; 1]';

% Q = C' * C + 0.001 * eye(2);
% R = 0.001;
Q = eye(2);
R = 1;

N = 5;
H = cell(N+1,1);   % Matlab starts from 1
H{N+1} = Q;
K = [];
u = [];
y = [];
x(:,1) = [0;1];

x_o(:,1) = [0;0];
u_o(1) = 1;
x_o(:,2) = A*x_o(:,1) + B*u_o(1);
for i = 2 : 35
    x_o(:,i+1) = A*x_o(:,i);
end
for i = 1:35+1
    y_o(i) = C * x_o(:,i);
end
plot(0:length(y_o)-1,y_o,'r-');