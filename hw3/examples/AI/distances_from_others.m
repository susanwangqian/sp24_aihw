function d = distances_from_others(x, N)
% FILE: distances_from_others.m - Finds the distances from two robots
%
% DESCRIPTION:
% Uses the positions of the robots and uses the distance formula to find
% the distance between two robots
%
% INPUTS:
% 1. x - the robots positions
% 2. N - Number of robots
%
% OUTPUTS:
% An N x N matrix representing the distance between the robots 
% 
%
% TODO:
% None

%% Authors: Safwan Alam, Musad Haque - 2018 
%%%%%%%%%%%%%

d = zeros(N,N);

%Distance formula
for i = 1:1:N
    for j = 1:1:N
        d(i,j) = sqrt(power((x(1, i) - x(1, j)), 2) +...
            power((x(2, i) - x (2, j)), 2));
    end
end

end


