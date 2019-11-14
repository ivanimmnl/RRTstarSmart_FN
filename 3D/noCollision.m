function bool = noCollision(node1,node2,obstacle)
%POINTSBETWEEN Summary of this function goes here
%   Detailed explanation goes here
bool = 1;

diff = [(node1(1) - node2(1)) (node1(2) - node2(2)) (node1(3) - node2(3))];
abs_diff = [abs(diff(1)) abs(diff(2)) abs(diff(3))];
[~, i] = max(abs_diff);
final_diff = diff(i);

% take the greater difference
switch i
    case 1
        dec_index1 = 2;
        dec_index2 = 3;
    case 2
        dec_index1 = 1;
        dec_index2 = 3;
    case 3
        dec_index1 = 1;
        dec_index2 = 2;
end

% Creates set of points between two points
points_to_check=[node2];
for ii = 1:abs(diff)
point(1) = node2(1) + ii*diff(1)/abs(final_diff);
point(2) = node2(2) + ii*diff(2)/abs(final_diff);
point(3) = node2(3) + ii*diff(3)/abs(final_diff);

temp1 = point(dec_index1);
temp2 = point(dec_index2);
point(dec_index1) = floor(point(dec_index1));
point(dec_index2) = floor(point(dec_index2));
points_to_check = cat(1, points_to_check, point);

if temp1 ~= point(dec_index1)
    point(dec_index1) = point(dec_index1) + 1;
    points_to_check = cat(1, points_to_check, point);
end

point(dec_index1) = point(dec_index1) - 1;
if temp2 ~= point(dec_index2)
    point(dec_index2) = point(dec_index2) + 1;
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

