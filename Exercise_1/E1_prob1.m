function E1_prob1(N)
%%%%%%% Exercise 1, Prob 1
    A = [4/3, -2/3; 1, 0];
    B = [1; 0];
    C = [-2/3; 1]';

    Q = C'*C + 0.001*eye(2);
    R = 0.001;
    Pf = Q;

    figure('name','output','Position',[1000 10 450 353]);
    figure('name','x','Position',[1000 5000 450 353]);

    % x_pre = [0;1];
    x(:,1) = [10;10];
    counter = 0;
    y = [];
    fig_points = [];
    fig_line = [];
%     while(1)
    for tim = 1:150
        H = cell(N+1,1);   % Matlab starts from 1
        H{N+1} = Pf;
        K = [];
        u = [];

        for i = N : -1 : 1
            K(i,:) = -inv(R + B'*H{i+1}*B)*B'*H{i+1}*A;
            H{i} = Q + K(i,:)'*R*K(i,:)+ (A+B*K(i,:))'*H{i+1}*(A+B*K(i,:));
        end

        for i = 1 : N
            u(i,:) = K(i,:) * x(:,counter + i);
            x(:,counter + i+1) = A*x(:,counter + i) + B*u(i,:);
        end
        x_pre = x(:,2);

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
        axis([-15 15 -15 15]);

        counter = counter + 1;
    end
end