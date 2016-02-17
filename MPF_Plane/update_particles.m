function [X, iffar] = update_particles(b_Xprior, b_Xpre, Xstd_ob, R,  M, N)

X = zeros(3,N);
i=0;
count=0;
factor=0;
nParticles=10000;
nIter=4;
iffar=0;
b_X=b_Xpre;
while i < N
    if count == nIter
        count=0;
        factor=factor+1;
        nParticles=10000*(10^factor);
        if(factor>3)
            iffar=1;
            b_X=b_Xprior;
            factor=0;
            nParticles=10000;
        end
    end
    tempa=b_X(1,1)+b_X(2,1) .* randn(nParticles,1);
    tempb=b_X(1,2)+b_X(2,2) .* randn(nParticles,1);
    tempc=b_X(1,3)+b_X(2,3) .* randn(nParticles,1);
    D = ((M(1,:)+tempa.*M(2,:)+tempb.*M(3,:)+tempc)./sqrt(1+tempa.^2+tempb.^2))-R;
    v=find(D >= -Xstd_ob & D <= Xstd_ob);
    i=length(v);
    count=count+1;
    if N/i>=2
        if N/i>10
            if N/i>100
                count=nIter;
                factor=factor+2;
            else
                count=nIter;
                factor=factor+1;
            end
        else
            count=nIter;
        end
    end
            
end
idx=randperm(i,N);
tempa=tempa(v);
tempb=tempb(v);
tempc=tempc(v);
X(1,:)=tempa(idx);
X(2,:)=tempb(idx);
X(3,:)=tempc(idx);

