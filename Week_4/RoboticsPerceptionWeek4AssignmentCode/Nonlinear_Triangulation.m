function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     x
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations 

Num = size(X0,1);

X = zeros(size(X0));
a_a_a = size(X);

for i = 1 : Num
    
    X_temp = X0(i,:);
    
    for j = 1 : 5
        X_temp = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1(i,:), x2(i,:), x3(i,:), X_temp);
    end
    a_a_a = size(X_temp);
    X(i,:) = X_temp;
end

end


function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
    
x1_p = K * R1*( X0' - C1);
x1_p = (x1_p ./ repmat(x1_p(3, :), [3, 1]))';

x2_p = K * R2 * (X0' - repmat(C2, [1 size(X0,1)]));
x2_p = (x2_p ./ repmat(x2_p(3, :), [3, 1]))';

x3_p = K * R3 * (X0' - repmat(C3, [1 size(X0,1)]));
x3_p = (x3_p ./ repmat(x3_p(3, :), [3, 1]))';


b = [ x1(1), x1(2), x2(1), x2(2), x3(1), x3(2) ]'; 
f_x = [ x1_p(1), x1_p(2), x2_p(1), x2_p(2), x3_p(1), x3_p(2) ]';

J =[ Jacobian_Triangulation(C1, R1, K, X0)',...
     Jacobian_Triangulation(C2, R2, K, X0)',...
     Jacobian_Triangulation(C3, R3, K, X0)' ]';

error = (x1(1)-x1_p(1))^2 + (x1(2)-x1_p(1))^2 + ...
        (x2(1)-x2_p(1))^2 + (x2(2)-x2_p(1))^2 + ...
        (x3(1)-x3_p(1))^2 + (x3(2)-x3_p(1))^2;
fprintf("Error: %f\n",error);

delta_X = (J'*J)\(J'*(b-f_x));


X = X0 + delta_X';

a_a_a = size(X0);
a_a_a = size(delta_X);
a_a_a = size(X);
end



function J = Jacobian_Triangulation(C, R, K, X)
    
x = K * R *( X' - C);

u = x(1);
v = x(2);
w = x(3);


f = K(1,1);
p_x = K(1,3);
p_y = K(2,3);


d_u = [ f*R(1,1)+p_x*R(3,1), f*R(1,2)+p_x*R(3,2), f*R(1,3)+p_x*R(3,3)];
d_v = [ f*R(2,1)+p_y*R(3,1), f*R(2,2)+p_y*R(3,2), f*R(2,3)+p_y*R(3,3)];
d_w = [ R(3,1), R(3,2), R(3,3)];


J = [(w*d_u-u*d_w)/(w^2);(w*d_v-v*d_w)/(w^2)];
end
