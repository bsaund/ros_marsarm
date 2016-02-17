function [ dis_transform, obstacle ] = distance_transform(box_para, range, res)

LARGE_NUM = 100000;
grid=(range(:,2)-range(:,1))/res;
dis_transform=zeros(size(grid));
obstacle=dis_transform;
for i = 1:grid(1)
    for j = 1:grid(2)
        for k = 1:grid(3)
            x=(i-1)*res+range(1,1);
            y=(j-1)*res+range(2,1);
            z=(k-1)*res+range(3,1);
            if(abs(x)<=box_para(1)/2 && abs(y) <= box_para(2)/2 && abs(z) <= box_para(3)/2)
                obstacle(i,j,k)=0;
            else
                obstacle(i,j,k)=LARGE_NUM;
            end
        end
    end
end
dis_transform=zeros(grid(1),grid(2),grid(3));
%g=zeros(1,grid(1));
for i = 1:grid(1)
    for j = 1:grid(2)
        dis_transform(i,j,:)=distance_transform_1D(grid(3), obstacle(i,j,:));
    end
    for k = 1:grid(3)
        dis_transform(i,:,k)=distance_transform_1D(grid(2), dis_transform(i,:,k));
    end
end
for j = 1:grid(2)
    for k = 1:grid(3)
        dis_transform(:,j,k)=sqrt(distance_transform_1D(grid(1), dis_transform(:,j,k))).*res;
    end
end
