function [X] = update_particles(Xstd_ob, R,  M, N)

X = zeros(3,N);
theta=pi.*rand(1,N);
phi=pi.*rand(1,N)+pi/2;
r=normrnd(R,Xstd_ob,1,N);
dir=[r.*sin(theta).*cos(phi);r.*sin(theta).*sin(phi);r.*cos(theta)];
pts=[dir(1,:)+M(1,:);dir(2,:)+M(2,:);dir(3,:)+M(3,:)];
X(1,:)=dir(2,:)./dir(1,:);
X(2,:)=dir(3,:)./dir(1,:);
X(3,:)=-pts(1,:)-X(1,:).*pts(2,:)-X(2,:).*pts(3,:);



