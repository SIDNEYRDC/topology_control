function dist = distance(i,j)
%DISTANCE calcule the euclidian distance between i and j
    dist = sqrt((i(1) - j(1))^2 + (i(2) - j(2))^2);
end

