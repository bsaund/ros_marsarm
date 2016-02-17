clear;

N = 500;

Xstd_ob = 0.0001;
Xstd_tran = 0.0025;
Xstd_scatter = 0.0001;

R=0.01;

Xprior=[2.1,1.4,0.8,pi/6, pi/12, pi/18];
Xprior_std=[0.01,0.01,0.01, pi/360, pi/360, pi/360];
X_true=[2.105,1.398,0.808, pi/6+pi/400, pi/12+pi/420, pi/18-pi/380];
box_para=[6,4,2];
b_Xprior=[Xprior;Xprior_std];
b_Xpre=b_Xprior;
X0 = create_particles(b_Xprior, N);
X_1=X0;
X_ini=X0;
N_Measure=90;
M_std=0.0001;
M=zeros(3,N_Measure);
for i = 1:N_Measure/3
    M(1,3*i-2)=normrnd(box_para(1)/2+R, M_std);
    M(2,3*i-2)=normrnd((rand-0.5)*3.8, M_std);
    M(3,3*i-2)=normrnd((rand-0.5)*1.8, M_std);
    M(1,3*i-1)=normrnd((rand-0.5)*5.6, M_std);
    M(2,3*i-1)=normrnd(box_para(2)/2+R, M_std);
    M(3,3*i-1)=normrnd((rand-0.5)*1.8, M_std);
    M(1,3*i)=normrnd((rand-0.5)*5.6, M_std);
    M(2,3*i)=normrnd((rand-0.5)*3.8, M_std);
    M(3,3*i)=normrnd(box_para(3)/2+R, M_std);  
end
Rot=[cos(X_true(6)),-sin(X_true(6)),0;sin(X_true(6)),cos(X_true(6)),0;0,0,1]*...
  [cos(X_true(5)),0,sin(X_true(5));0,1,0;-sin(X_true(5)),0,cos(X_true(5))]*...
  [1,0,0;0,cos(X_true(4)),-sin(X_true(4));0,sin(X_true(4)),cos(X_true(4))];
% fig=figure;
% scatter3(M(1,:),M(2,:),M(3,:),'.');
% hold;
for i = 1:N_Measure
    temp =Rot*[M(1,i),M(2,i),M(3,i)]';
    M(1,i)=temp(1)+X_true(1);
    M(2,i)=temp(2)+X_true(2);
    M(3,i)=temp(3)+X_true(3);
end
X_est=zeros(8,N_Measure);

% fig=figure(1);
% scatter3(X0(1,:),X0(2,:),X0(3,:),'.');
% hold;
% plot3(X_true(1),X_true(2),X_true(3),'.r','LineWidth',4,'MarkerSize',20);
% % axis([1.6 2.4 1 1.8 3.6 4.4]);
% view(47,22);
% hold;

for k = 1:60
    if k>1
        %temp=X0;
        X0(:,:)=X0(:,:)+Xstd_scatter.*randn(6,N);        
        b_Xpre(1,1:6)=mean(X0,2)';
        %b_Xpre(2,1:6)=[sqrt(sum((X0(1,:)-b_Xpre(1, 1)).^2)/N),sqrt(sum((X0(2,:)-b_Xpre(1, 2)).^2)/N),sqrt(sum((X0(3,:)-b_Xpre(1, 3)).^2)/N)];
    end
    [X, iffar] = update_particles(X_1, X0, Xstd_ob, Xstd_tran, R, box_para, M(:,k), k);
    if k==1
        X0=X;
    elseif iffar == 1
        X0=X_1;
    end
    
%     scatter3(X(1,:),X(2,:),X(3,:),'.');
%     hold;
%     plot3(X_est(1, k),X_est(2, k),X_est(3, k),'+k');
%     plot3(X_true(1),X_true(2),X_true(3),'.r','LineWidth',4,'MarkerSize',20);
%     %axis([1.6 2.4 1 1.8 3.6 4.4]);
%     view(47,22);
%     hold;

    %W =  calc_weight(Xstd_tran, X0(1:3, :), X(1:3, :));
    X_1=X0;
    %X0 = resample_particles(X, W);
    X0=X;
    
    X_est(1:6 ,k) = mean(X0,2);
    %X_est(7,k) = sum(((M(1,1:k)+X_est(1, k)*M(2,1:k)+X_est(2, k)*M(3,1:k)+X_est(3, k))/sqrt(1+X_est(1, k)^2+X_est(2, k)^2)-R).^2)/k;
    X_est(7,k)=sqrt((X_true(1)-X_est(1, k)).^2+(X_true(2)-X_est(2, k)).^2+(X_true(3)-X_est(3, k)).^2+(X_true(4)-X_est(4, k)).^2+(X_true(5)-X_est(5, k)).^2+(X_true(6)-X_est(6, k)).^2);
    X_est(8,k) = sqrt(sum((X0(1,:)-X_est(1, k)).^2+(X0(2,:)-X_est(2, k)).^2+(X0(3,:)-X_est(3, k)).^2+(X0(4,:)-X_est(4, k)).^2+(X0(5,:)-X_est(5, k)).^2+(X0(6,:)-X_est(6, k)).^2)/N);
    if X_est(8,k)<0.005 && any(find(abs(X_est(1:6,k)'-b_Xpre(1,:))>0.001))
        Xstd_scatter=0.01;
    else
        Xstd_scatter=0.0001;
    end
    
    figure(01);
    clf;
    scatter3(X0(1,:),X0(2,:),X0(3,:),'.');
    hold;
    plot3(X_true(1),X_true(2),X_true(3),'.r','LineWidth',4,'MarkerSize',20);
    plot3(X_est(1, k),X_est(2, k),X_est(3, k),'+k');
%     axis([1.6 2.4 1 1.8 3.6 4.4]);
    view(47,22);
    hold;
    
    
    figure(02);
    clf;
    scatter3(X0(4,:),X0(5,:),X0(6,:),'.');
    hold;
    plot3(X_true(4),X_true(5),X_true(6),'.r','LineWidth',4,'MarkerSize',20);
    plot3(X_est(4, k),X_est(5, k),X_est(6, k),'+k');
%     axis([1.6 2.4 1 1.8 3.6 4.4]);
    view(47,22);
    hold;
end

