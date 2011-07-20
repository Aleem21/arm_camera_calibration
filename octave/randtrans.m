function T = randtrans()
    R = rand(3,3);
    [u s v] = svd(R);
    if (det(u) * det(v) < 0)
        R = u * -v';
    else
        R = u * v';
    end
    t = rand(3,1);
    T = [R,t];
    T = [T
         0 0 0 1];
end
