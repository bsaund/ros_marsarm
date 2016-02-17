clear;

N = 900;

Xstd_ob = 0.0001;
Xstd_tran = 0.0025;
Xstd_scatter = 0.0001;

R=0.01;
r=5;
Xprior=[2,1.4,4,0.06,0.04];
Xprior_std=[0.1,0.1,0.1,0.1,0.1];
X_true=[2.08,1.43,3.94,0,0];
tempf=-(X_true(4)+X_true(1)*X_true(5)+X_true(3))/X_true(2);
b_Xprior=[Xprior;Xprior_std];
b_Xpre=b_Xprior;
X0 = create_particles(b_Xprior, N*40);
Xpre_weight=X0;
N_Measure=50;
M_std=0.000;
M=zeros(3,N_Measure);
M_label=zeros(1,N_Measure);
i=1;
while i <= N_Measure
    while 1
        M(1,i)=(rand-0.5)*8;
        M(2,i)=(rand-0.5)*8;
        M(3,i)=(R*(1+X_true(1)^2+X_true(2)^2).^0.5-X_true(3)-M(1,i)-X_true(1)*M(2,i))/X_true(2);
        if (((M(1,i)-X_true(4))^2+(M(2,i)-X_true(5))^2+(M(3,i)-tempf)^2)<=(r^2+R^2))
            M(3,i)=M(3,i)+M_std* randn;
            M_label(i)=0;
            break;
        else
            while 1
                M(1,i)=(rand-0.5)*8;
                M(2,i)=(rand-0.5)*8;
                M(3,i)=(R*2/3.0*(1+X_true(1)^2+X_true(2)^2).^0.5-X_true(3)-M(1,i)-X_true(1)*M(2,i))/X_true(2);
                D2_2=((M(1,i)-X_true(4))^2+(M(2,i)-X_true(5))^2+(M(3,i)-tempf)^2);                
                if (((D2_2-(R*2/3.0)^2)^0.5-(R^2-(R*2/3.0)^2)^0.5-r<=Xstd_ob/2) && ((D2_2-(R*2/3.0)^2)^0.5-(R^2-(R*2/3.0)^2)^0.5-r>=-Xstd_ob/2))
                    M_label(i)=1;
                    break;
                end
            end
            break;
        end
    end
    M(1,i)=M(1,i)+M_std* randn;
    M(2,i)=M(2,i)+M_std* randn;
    
    i=i+1
end
X_est=zeros(7,N_Measure);

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
        X0(:,:)=X0(:,:)+Xstd_scatter.*randn(5,N);        
        b_Xpre(1,1:5)=mean(X0,2)';
        b_Xpre(2,1:5)=[sqrt(sum((X0(1,:)-b_Xpre(1, 1)).^2)/N),sqrt(sum((X0(2,:)-b_Xpre(1, 2)).^2)/N),sqrt(sum((X0(3,:)-b_Xpre(1, 3)).^2)/N),sqrt(sum((X0(4,:)-b_Xpre(1, 4)).^2)/N),sqrt(sum((X0(5,:)-b_Xpre(1, 5)).^2)/N)];
    end
    [X, iffar] = update_particles(b_Xprior, b_Xpre, Xstd_ob, R, r, M(:,k), N);
    if k==1
        X0=X;
    elseif iffar == 1
        X0=Xpre_weight;
    end
    
    scatter3(X(1,:),X(2,:),X(3,:),'.');
    hold;
    plot3(X_est(1, k),X_est(2, k),X_est(3, k),'+k');
    plot3(X_true(1),X_true(2),X_true(3),'.r','LineWidth',4,'MarkerSize',20);
    axis([1.6 2.4 1 1.8 3.6 4.4]);
    view(47,22);
    hold;

    W =  calc_weight(Xstd_tran, X0(1:5, :), X(1:5, :));
    Xpre_weight=X0;
    X0 = resample_particles(X, W);
    
    X_est(1:5,k) = mean(X0,2);
    %X_est(6,k) = sum(((M(1,1:k)+X_est(1, k)*M(2,1:k)+X_est(2, k)*M(3,1:k)+X_est(3, k))/sqrt(1+X_est(1, k)^2+X_est(2, k)^2)-R).^2)/k;
    X_est(7,k) = sqrt(sum((X0(1,:)-X_est(1, k)).^2+(X0(2,:)-X_est(2, k)).^2+(X0(3,:)-X_est(3, k)).^2+(X0(4,:)-X_est(4, k)).^2+(X0(5,:)-X_est(5, k)).^2)/N);
    if X_est(7,k)<0.025 && any(find(abs(X_est(1:5,k)'-b_Xpre(1,:))>0.001))
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

