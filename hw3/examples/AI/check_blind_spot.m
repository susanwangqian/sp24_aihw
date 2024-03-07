function [blind_spot_neighbors] = check_blind_spot(x, N, half_sense)
% FILE: check_blind_spot.m creates a list of agents in your blindspot
%
% DESCRIPTION: checks whether the size of the angle made between agent i
% and j is less than half the size of the sensing sector of agent i
%
% INPUTS:
% 1. x - the pose of all Robotarium robots
% 2. N - the number of RObotarium robots
% 3. half_sense - half the size of each agents's sensing sector
%
% OUTPUTS:
% An N x N matrix where a 1 in position (row=i, col=j) encodes that agent j
% is in agent i's blindspot
% 
%
% TODO:
% None

%% Authors: Musad Haque, Safwan Alam - 2019
%%%%%%%%%%%%%

blind_spot_neighbors = zeros(N, N);

for i=1:1:N
    for j=1:1:N
        if (i ~= j)
            angle_ij = atan2((x(2,j) - x(2,i)),(x(1,j) - x(1,i)));
            angle_diff = x(3,i) - angle_ij;
            angle_diff_norm = mod(angle_diff, 2*pi);
            if min(2*pi-angle_diff_norm, angle_diff_norm) > half_sense
                blind_spot_neighbors(i,j) = 1;
            end
        end
    end
end


