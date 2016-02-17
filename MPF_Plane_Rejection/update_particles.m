function [X, iffar] = update_particles(X_1, X0, Xstd_ob, Xstd_tran, R,  M)
N=length(X0(1,:));
X = zeros(3,N);
count=0;
i=0;
iffar=0;
b_X=X0;
while count < N
    if((i>=1000000 || (count>0 && i/count>5000))&& iffar==0)
        iffar=1;
        b_X=X_1;
        count=0;
        i=0;
    end        
    idx=randi(N);
    tempa=normrnd(b_X(1,idx),Xstd_tran);
    tempb=normrnd(b_X(2,idx),Xstd_tran);
    tempc=normrnd(b_X(3,idx),Xstd_tran);
    D = ((M(1,:)+tempa.*M(2,:)+tempb.*M(3,:)+tempc)./sqrt(1+tempa.^2+tempb.^2))-R;
    if (D >= -Xstd_ob && D <= Xstd_ob)
        count=count+1;
        X(1,count)=tempa;
        X(2,count)=tempb;
        X(3,count)=tempc;
    end
    i=i+1;
end



