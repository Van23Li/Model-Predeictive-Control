%%%%%%% Computation of LQR control laws using Matlab’s function

clear all
close all
A = [1.988,-0.998;1,0];
B = [0.125; 0];
C = [-2/3; 1]';

Q = eye(2);
R = 1;

[K,S,e] = dlqr(A,B,Q,R);

figure('name','output','Position',[1000 10 450 353]);
figure('name','x','Position',[1000 5000 450 353]);

x(:,1) = [0;1];
counter = 0;
y(:,1) = C * x(:,1);
fig_points = [];
fig_line = [];
while(1)
    u = - K * x(:,counter+1);
    x(:,counter+2) = A * x(:,counter+1) + B * u;
    
    y(:,counter+2) = C * x(:,counter+2);
    
    % plot output
    figure(1)
    plot(0:length(y)-1,y,'r-');
    xlim([0 150]);
    
    % plot state
    figure(2)
    plot(x(1,:),x(2,:),'or');
    axis([-3 3 -2 2]);
    
    counter = counter + 1; 
end