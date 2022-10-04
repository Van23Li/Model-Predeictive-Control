%%%%%%% Exercise 1, Prob 3
clear all
close all

A = [4/3, -2/3; 1, 0];
B = [1; 0];
C = [-2/3; 1]';

Q = C'*C + 0.001*eye(2);
R = 0.001;
Pf = Q;
[K,S,e] = dlqr(A,B,Q,R);

figure('name','output','Position',[1000 10 450 353]);
figure('name','x','Position',[1000 5000 450 353]);

x(:,1) = [10;10];
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
    axis([-15 15 -15 15]);
    
    counter = counter + 1; 
end