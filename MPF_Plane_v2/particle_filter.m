clear;

N = 100000;

Xstd_ob = 0.0001;
Xstd_tran = 0.0025;
Xstd_scatter = 0.0001;

R=0.01;

Xprior=[2,1.4,4];
Xprior_std=[0.1,0.1,0.1];
X_true=[2.1,1.45,3.9];

b_Xprior=[Xprior;Xprior_std];
b_Xpre=b_Xprior;
X0 = create_particles(b_Xprior, N*40);
Xpre_weight=X0;
N_Measure=100;
M_std=0.000;
M=zeros(3,N_Measure);
for i = 1:N_Measure
    M(1,i)=(rand-0.5)*6;
    M(2,i)=(rand-0.5)*6;
    M(3,i)=(R*(1+X_true(1)^2+X_true(2)^2).^0.5-X_true(3)-M(1,i)-X_true(1)*M(2,i))/X_true(2)+M_std* randn;
    M(1,i)=M(1,i)+M_std* randn;
    M(2,i)=M(2,i)+M_std* randn;
end
X_est=zeros(5,N_Measure);

fig=figure;
scatter3(X0(1,:),X0(2,:),X0(3,:),'.');
hold;
plot3(X_true(1),X_true(2),X_true(3),'.r','LineWidth',4,'MarkerSize',20);
axis([1.6 2.4 1 1.8 3.6 4.4]);
view(47,22);
hold;

for k = 1:N_Measure
    if k>1
        %temp=X0;
        X0(:,:)=X0(:,:)+Xstd_scatter.*randn(3,N);        
        b_Xpre(1,1:3)=mean(X0,2)';
        b_Xpre(2,1:3)=[sqrt(sum((X0(1,:)-b_Xpre(1, 1)).^2)/N),sqrt(sum((X0(2,:)-b_Xpre(1, 2)).^2)/N),sqrt(sum((X0(3,:)-b_Xpre(1, 3)).^2)/N)];
    end
    [X] = update_particles(Xstd_ob, R, M(:,k), N);
    if k==1
        X0=X;
    end
    
    scatter3(X(1,:),X(2,:),X(3,:),'.');
    hold;
    plot3(X_est(1, k),X_est(2, k),X_est(3, k),'+k');
    plot3(X_true(1),X_true(2),X_true(3),'.r','LineWidth',4,'MarkerSize',20);
    axis([1.6 2.4 1 1.8 3.6 4.4]);
    view(47,22);
    hold;

    W =  calc_weight(Xstd_tran, X0(1:3, :), X(1:3, :));
    Xpre_weight=X0;
    X0 = resample_particles(X, W);
    
    X_est(1:3,k) = mean(X0,2);
    X_est(4,k) = sum(((M(1,1:k)+X_est(1, k)*M(2,1:k)+X_est(2, k)*M(3,1:k)+X_est(3, k))/sqrt(1+X_est(1, k)^2+X_est(2, k)^2)-R).^2)/k;
    X_est(5,k) = sqrt(sum((X0(1,:)-X_est(1, k)).^2+(X0(2,:)-X_est(2, k)).^2+(X0(3,:)-X_est(3, k)).^2)/N);
    if X_est(5,k)<0.005 && any(find(abs(X_est(1:3,k)'-b_Xpre(1,:))>0.001))
        Xstd_scatter=0.01;
    else
        Xstd_scatter=0.0001;
    end
        
    scatter3(X0(1,:),X0(2,:),X0(3,:),'.');
    hold;
    plot3(X_true(1),X_true(2),X_true(3),'.r','LineWidth',4,'MarkerSize',20);
    plot3(X_est(1, k),X_est(2, k),X_est(3, k),'+k');
    axis([1.6 2.4 1 1.8 3.6 4.4]);
    view(47,22);
        hold;
end

