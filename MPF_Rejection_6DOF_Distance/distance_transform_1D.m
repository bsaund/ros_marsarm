function [Df] = distance_transform_1D(range, f)
Df=zeros(1, range);
idx=1;
envelope(1)=1;
bound(1)=-inf;
bound(2)=inf;
for k = 2:range
    s=((f(k)+k^2)-(f(envelope(idx))+envelope(idx)^2))/(2*k-2*envelope(idx));
    while s <= bound(idx)
        idx = idx - 1;
        s=((f(k)+k^2)-(f(envelope(idx))+envelope(idx)^2))/(2*k-2*envelope(idx));
    end
    idx = idx + 1;
    envelope(idx) = k;
    bound(idx) = s;
    bound(idx+1)=inf;
end
idx=1;
for k = 1:range
    while bound(idx+1) < k
        idx = idx + 1;
    end
    Df(k)=(k-envelope(idx))^2+f(envelope(idx));
end