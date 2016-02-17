Pos=[2.08,1.43,3.94];
R=0;
% M(1)=-9;
% M(2)=3;
% M(3)=(R*(1+Pos(1)^2+Pos(2)^2).^0.5-Pos(3)-M(1)-Pos(1)*M(2))/Pos(2);
% point=M;

%[xx,yy]=meshgrid(1.6:0.001:2.4,1:0.001:1.8);
[xx,yy]=meshgrid(-20:0.1:20,-20:0.1:20);
%xx=rand(20000,1)*0.8+1.6;
%yy=rand(20000,1)*0.8+1;
%C=R.*(1+xx.^2+yy.^2).^0.5-point(1)-xx.*point(2)-yy.*point(3);
zz=(-xx-Pos(1).*yy-Pos(3))./Pos(2);
mesh(xx,yy,zz);
%scatter3(xx,yy,C,'.');
hold;
plot3(M(1,:),M(2,:),M(3,:),'.k');

%axis([1.6 2.4 1 1.8 3.6 4.4]);
view(47,22);
hold;
%mesh(xx,yy,0.*(1+xx.^2+yy.^2).^0.5-point(1)-xx.*point(2)-yy.*point(3));
%axis([1.6 2.4 1 1.8 3.6 4.4]);