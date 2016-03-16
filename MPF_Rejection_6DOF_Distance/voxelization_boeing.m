[v, f, n, c, stltitle] = stlread('boeing_part_binary.stl');
v = v.*0.0254;
scatter3(v(:,1),v(:,2),v(:,3),'.');
hold;
% plot3(X_est(1, k),X_est(2, k),X_est(3, k),'+k');
% plot3(X_true(1),X_true(2),X_true(3),'.r','LineWidth',4,'MarkerSize',20);
%axis([1.6 2.4 1 1.8 3.6 4.4]);
view(47,22);
%hold;
R = 0.001;
Xstd_ob = 0.001;
voxel_size = 0.001;
bbox = zeros(2,3);
c_touch = [1.125, 0.23, 0.1];
c_size = 0.04;
hashmap = containers.Map;
for i = -1:2:1
    for j = -1:2:1
        plot3([c_touch(1) + i*c_size/2, c_touch(1) + i*c_size/2],[c_touch(2) + j*c_size/2,c_touch(2)+ j*c_size/2],[c_touch(3) - c_size/2,c_touch(3) + c_size/2], 'k');
    end
end
for i = -1:2:1
    for j = -1:2:1
        plot3([c_touch(1) + i*c_size/2, c_touch(1) + i*c_size/2],[c_touch(2) - c_size/2,c_touch(2) + c_size/2],[c_touch(3) + j*c_size/2,c_touch(3) + j*c_size/2], 'k');
    end
end
for i = -1:2:1
    for j = -1:2:1
        plot3([c_touch(1) - c_size/2, c_touch(1) + c_size/2],[c_touch(2) + j*c_size/2,c_touch(2)+ j*c_size/2],[c_touch(3) + i*c_size/2,c_touch(3) + i*c_size/2], 'k');
    end
end
plot3(c_touch(1),c_touch(2),c_touch(3),'.k','MarkerSize',20);
for i=1:length(n(:,1))
   
    temp = min(v(3*i - 2:3*i,:), [], 1) - 2*R;
    bbox(1,:) = temp - mod(temp, voxel_size) - voxel_size / 2;
    temp = max(v(3*i - 2:3*i,:), [], 1) + 2*R;
    bbox(2,:) = temp - mod(temp, voxel_size) + 3 * voxel_size / 2;
    if ((bbox(1,1) > (c_touch(1) + c_size/2)) || (bbox(1,2) > (c_touch(2) + c_size/2)) || (bbox(1,3) > (c_touch(3) + c_size/2)) ...
        || (bbox(2,1) < (c_touch(1) - c_size/2)) || (bbox(2,2) < (c_touch(2) - c_size/2)) || (bbox(2,3) < (c_touch(3) - c_size/2)))
        continue;
    end
    i;  
    pstart = max(bbox(1,:), c_touch - c_size/2);
    pend = min(bbox(2,:), c_touch + c_size/2);
    v0 = v(3*i-2,:);
    v1 = v(3*i-1,:);
    v2 = v(3*i,:);
    w0 = v1-v0;
    w1 = v2 - v0;
    d00 = dot(w0,w0);
    d01 = dot(w0,w1);
    d11 = dot(w1,w1);
    denom = d00 *d11 - d01*d01;
    n(i,:) = cross(w0,w1)./norm(cross(w0,w1));
    plot3([v0(1),v1(1)],[v0(2),v1(2)],[v0(3),v1(3)], 'b');
    plot3([v1(1),v2(1)],[v1(2),v2(2)],[v1(3),v2(3)], 'b');
    plot3([v0(1),v2(1)],[v0(2),v2(2)],[v0(3),v2(3)], 'b');
    
    %triArea = norm(cross(v1 - v0, v2 - v0)) * 0.5;
    for ix = pstart(1):voxel_size:pend(1)
        for iy = pstart(2):voxel_size:pend(2)
            for iz = pstart(3):voxel_size:pend(3)
                dist = dot(n(i,:), [ix,iy,iz] - v(3*i-2,:));
                if (dist >= R - Xstd_ob && dist <= R + Xstd_ob)                   
                    p=[ix,iy,iz] - dist .*n(i,:);
                    w2 = p - v0;
                    d20 = dot(w2,w0);
                    d21 = dot(w2,w1);
                    alpha = (d11*d20 - d01*d21)/denom;%(norm(cross((v1 - p)', (v2 - p)')) * 0.5) / triArea;
                    beta = (d00*d21 - d01*d20)/denom;%(norm(cross((v2 - p)', (v0 - p)')) * 0.5) / triArea;
                    gamma = 1 - alpha - beta;
                    if (alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1 && gamma >= 0 && gamma <= 1)                   
                        key = [num2str(ix - voxel_size / 2), ' ', num2str(iy - voxel_size / 2), ' ', num2str(iz - voxel_size / 2)];
                        if ((~isKey(hashmap,key)) || (isKey(hashmap,key) && ~strcmp(hashmap(key),'-1')))
                            hashmap(key) = '1';
                            %if (i == 5)
                                %plot3(ix,iy,iz,'.r');
%                             elseif(i == 6)
%                                 plot3(ix,iy,iz,'.g');
%                             elseif(i == 7)
%                                 plot3(ix,iy,iz,'.b');
%                             elseif(i == 8)
%                                 plot3(ix,iy,iz,'.c');
%                             elseif(i == 9)
%                                 plot3(ix,iy,iz,'.k');
%                             elseif(i == 10)
%                                 plot3(ix,iy,iz,'.y');
%                             end
                        end
                    end
                elseif(dist <  R - Xstd_ob)
                    p=[ix,iy,iz] - dist .*n(i,:);
                    w2 = p - v0;
                    d20 = dot(w2,w0);
                    d21 = dot(w2,w1);
                    alpha = (d11*d20 - d01*d21)/denom;
                    beta = (d00*d21 - d01*d20)/denom;
                    gamma = 1 - alpha - beta;
                    if (alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1 && gamma >= 0 && gamma <= 1)
                        key = [num2str(ix - voxel_size / 2), ' ', num2str(iy - voxel_size / 2), ' ', num2str(iz - voxel_size / 2)];
                        if(dist < -R && (~isKey(hashmap,key) || (isKey(hashmap,key) && ~strcmp(hashmap(key),'-1') && ~strcmp(hashmap(key),'1') )))
                            hashmap(key)  = '-2';
                        else
                            hashmap(key)  = '-1';
                        end
                    end
                end
            end
        end
    end                    
end
keyset = keys(hashmap);
len = length(hashmap);
for i=1:len
    key = keyset{i};
    if(strcmp(hashmap(key), '1'))
        temp = (strsplit(key, ' '));      
        plot3(str2num(temp{1}),str2num(temp{2}),str2num(temp{3}),'.r');
    end
end