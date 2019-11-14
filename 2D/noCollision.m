function bool = noCollision(node1,node2,obstacle)
%POINTSBETWEEN Summary of this function goes here
%   Detailed explanation goes here
bool = 1;

diff1 = (node1(1) - node2(1));
diff2 = (node1(2) - node2(2));

% take the greater difference
if abs(diff1) > abs(diff2)
    diff = diff1;
    decimated_index = 2;
else
    diff = diff2;
    decimated_index = 1;
end

% Creates set of points between two points
points_to_check=[node2];
for ii = 1:abs(diff)
point(1) = node2(1) + ii*diff1/abs(diff);
point(2) = node2(2) + ii*diff2/abs(diff);

temp = point(decimated_index);
point(decimated_index) = floor(point(decimated_index));
points_to_check = cat(1, points_to_check, point);

if temp ~= point(decimated_index)
    point(decimated_index) = point(decimated_index) + 1;
    points_to_check = cat(1, points_to_check, point);
end

end

% returns false if one of the point in between is an obstacle
for jj = 1:(size(points_to_check,1)-1)
    if(~ObstacleFree(points_to_check(jj,:), obstacle))
        bool = 0;
    end
end
end

