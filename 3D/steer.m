function A = steer(qr, qn, val, eps)
   qnew = [0 0 0];
   
   % Steer towards qn with maximum step size of eps
   if val >= eps
       qnew(1) = qn(1) + floor(((qr(1)-qn(1))*eps)/dist(qr,qn));
       qnew(2) = qn(2) + floor(((qr(2)-qn(2))*eps)/dist(qr,qn));
       qnew(3) = qn(3) + floor(((qr(3)-qn(3))*eps)/dist(qr,qn));
   else
       qnew(1) = qr(1);
       qnew(2) = qr(2);
       qnew(3) = qr(3);
   end   
   A = [qnew(1), qnew(2), qnew(3)];
end