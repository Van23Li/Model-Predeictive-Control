%%%%%%% Computation of finite-horizon LQR control laws using dynamic programming

clear all
close all
A = [1.988,-0.998;1,0];
B = [0.125; 0];
C = [-2/3; 1]';

Q = eye(2);
R = 1;

N = 5;

figure('name','output','Position',[1000 10 450 353]);
figure('name','x','Position',[1000 5000 450 353]);

% matrix
A_M = kron(diag(diag(eye(N-1)),-1),A);
A_M = A_M - eye(length(A_M));
B_M = kron(diag(diag(eye(N))),B);
C_int = zeros(N,1);
C_int(1) = 1;
C_M = kron(C_int,-A);
Q_M = kron(diag(diag(eye(N))),Q);
R_M = kron(diag(diag(eye(N))),R);

% calculate F and G
F = -inv(A_M) * B_M;
G = inv(A_M) * C_M;
K_M = -inv(R_M + F' * Q_M * F) * F' * Q_M * G;


figure('name','output','Position',[1000 10 450 353]);
figure('name','x','Position',[1000 5000 450 353]);

x(:,1) = [0;1];
counter = 0;
y = [];
fig_points = [];
fig_line = [];
while(1)
    u = K_M * x(:, counter + 1);
    x_pred = F * u + G * x(:, counter + 1);
    x = [x(:,1:counter + 1),reshape(x_pred',2,[])];
    
    for i = 1 : N+1
        y(counter + i) = C*x(:,counter + i);
    end
    
    % plot output
    figure(1)
    if ~isempty(fig_points)
        set(fig_line,'visible','off');
    end
    hold on
    plot(0:length(y(1:counter+2))-1,y(1:counter+2),'r-');
    fig_line = plot(length(y(1:counter+2))-1:length(y(1:counter+2))-1+length(y(counter+2:end))-1,y(counter+2:end),'b-');
    hold off
    xlim([0 150]);
    
    % plot state
    figure(2)
    if ~isempty(fig_points)
        set(fig_points,'visible','off');
    end
    hold on
    plot(x(1,1:counter+2),x(2,1:counter+2),'or');
    fig_points = plot(x(1,counter+3:end),x(2,counter+3:end),'ob');
    hold off
    axis([-3 3 -2 2]);
    
    counter = counter + 1; 
end