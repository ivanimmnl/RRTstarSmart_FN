function angle = angle_between(a, b)
    angle = (180*acos(dot(a,b)/norm(a)*norm(b)))/pi;
end