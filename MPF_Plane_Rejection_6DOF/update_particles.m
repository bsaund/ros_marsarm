function [X, iffar] = update_particles(X_1, X0, Xstd_ob, Xstd_tran, R, box_para, M, k)
N=length(X0(1,:));
X = zeros(3,N);
count=0;
i=0;
iffar=0;
b_X=X0;
normvec=zeros(3,1);
dir=1;
if rem(k,3) == 1
    normvec = [box_para(1)/2;0;0];
    dir=1;
elseif rem(k,3) == 2
    normvec = [0;box_para(2)/2;0];
    dir=2;
else
    normvec = [0;0;box_para(3)/2];
    dir=3;
end
    
while count < N
    if((i>=10000000 || (count>0 && i/count>5000))&& iffar==0)
        iffar=1;
        b_X=X_1;
        count=0;
        i=0;
    end        
    idx=randi(N);
    tempa=normrnd(b_X(1,idx),Xstd_tran);
    tempb=normrnd(b_X(2,idx),Xstd_tran);
    tempc=normrnd(b_X(3,idx),Xstd_tran);
    tempd=normrnd(b_X(4,idx),Xstd_tran);
    tempe=normrnd(b_X(5,idx),Xstd_tran);
    tempf=normrnd(b_X(6,idx),Xstd_tran);
    rot=[cos(tempf),-sin(tempf),0;sin(tempf),cos(tempf),0;0,0,1]*...
        [cos(tempe),0,sin(tempe);0,1,0;-sin(tempe),0,cos(tempe)]*...
        [1,0,0;0,cos(tempd),-sin(tempd);0,sin(tempd),cos(tempd)];
    vec=rot*normvec;
    M_inv = rot'* (M-[tempa;tempb;tempc]);
    D = M_inv(dir)-box_para(dir)/2-R;
    if (D >= -Xstd_ob && D <= Xstd_ob)
        count=count+1;
        X(1,count)=tempa;
        X(2,count)=tempb;
        X(3,count)=tempc;
        X(4,count)=tempd;
        X(5,count)=tempe;
        X(6,count)=tempf;
    end
    i=i+1;
end



