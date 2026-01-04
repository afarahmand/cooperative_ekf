% Creates an Identity Matrix

function [identity_matrix] = I(n)
   identity_matrix = zeros(n, n);
   for (count = 1:n)       identity_matrix(count, count) = 1;   end
end