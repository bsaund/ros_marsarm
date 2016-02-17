clear;

N = 8000;

Xstd_dis = 0.001;
Xstd_pos = 0.001;

R=0.01;

Trgt=[2,1.4,4];
Trgt_std=[0.1,0.1,0.1];
Pos=[2.1,1.45,3.9];

D_Trgt=[Trgt;Trgt_std];
X = create_particles(D_Trgt, N);

N_Measure=100;
M_std=0.001;
M=zeros(3,N_Measure);
for i = 1:N_Measure
    M(1,i)=(rand-0.5)*6;
    M(2,i)=(rand-0.5)*6;
    %M(1,i)=i/10+M_std* randn;
    %M(2,i)=(-1)^i*i/10+M_std* randn;
    M(3,i)=(R*(1+Pos(1)^2+Pos(2)^2).^0.5-Pos(3)-M(1,i)-Pos(1)*M(2,i))/Pos(2)+M_std* randn;
    M(1,i)=M(1,i)+M_std* randn;
    M(2,i)=M(2,i)+M_std* randn;
end
X_est=zeros(5,N_Measure);
X_origin = mean(X,2);
%fig=figure;
%scatter3(X(1,:),X(2,:),X(3,:),'.');
%hold;
%plot3(Pos(1),Pos(2),Pos(3),'.r','LineWidth',4,'MarkerSize',20);
%axis([1.6 2.4 1 1.8 3.6 4.4]);
%view(47,22);
%hold;
for k = 1:N_Measure
    X = update_particles(Xstd_pos, X);
%    scatter3(X(1,:),X(2,:),X(3,:),'.');
%    hold;
%    plot3(X_est(1, k),X_est(2, k),X_est(3, k),'+k');
%    plot3(Pos(1),Pos(2),Pos(3),'.r','LineWidth',4,'MarkerSize',20);
%    axis([1.6 2.4 1 1.8 3.6 4.4]);
%    view(47,22);
%    hold;
    L = calc_log_likelihood(Xstd_dis, R, X(1:3, :), M(:,k));
    X = resample_particles(X, L);
    X_est(1:3,k) = mean(X,2);
    X_est(4,k) = sum(((M(1,1:k)+X_est(1, k)*M(2,1:k)+X_est(2, k)*M(3,1:k)+X_est(3, k))/sqrt(1+X_est(1, k)^2+X_est(2, k)^2)-R).^2)/k;
    X_est(5,k) = sum((X(1,:)-X_est(1, k)).^2+(X(2,:)-X_est(2, k)).^2+(X(3,:)-X_est(3, k)).^2)/N;
    
    scatter3(X(1,:),X(2,:),X(3,:),'.');
    hold;
    plot3(Pos(1),Pos(2),Pos(3),'.r','LineWidth',4,'MarkerSize',20);
    plot3(X_est(1, k),X_est(2, k),X_est(3, k),'+k');
    axis([1.6 2.4 1 1.8 3.6 4.4]);
    view(47,22);
    hold;
end

