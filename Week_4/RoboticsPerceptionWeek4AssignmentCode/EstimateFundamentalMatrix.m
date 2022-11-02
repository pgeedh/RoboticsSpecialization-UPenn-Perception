function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2

[Num, ~] = size(x1);

A = zeros(Num, 9);

for i = 1:Num
    A(i, 1) = x1(i, 1)*x2(i, 1);
    A(i, 2) = x1(i, 1)*x2(i, 2);
    A(i, 3) = x1(i, 1);
    A(i, 4) = x1(i, 2)*x2(i, 1);
    A(i, 5) = x1(i, 2)*x2(i, 2);
    A(i, 6) = x1(i, 2);
    A(i, 7) = x2(i, 1);
    A(i, 8) = x2(i, 2);
    A(i, 9) = 1;
end


[~, ~, V] = svd(A);
x = V(:,end);
F = reshape(x, 3, 3)';


[u, d, v] = svd(F);% SVD of Fundamental matrix
d(3, 3) = 0;

F = u*d*v';
F = F/norm(F);
end
