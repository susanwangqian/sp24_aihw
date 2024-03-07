function vel = swarm(rad_rep, rad_ori, rad_att, x, N, dxi)
% FILE: swarm.m implements a Boids-like behavior
%
% DESCRIPTION:
% Boids-like repulsion-orientation-attraction behavior based loosely on the 
% behavior described by Couzin et al. in the Collective Memory paper. 
%
% INPUTS:
% rad_rep - radius of repulsion
% rad_ori - radius of orientation
% rad_att - radius of attraction
% x - matrix containing the pose of all the robots; x(1, ii) is the
% position of robot ii along the horizontal axis; x(2, ii) is the position
% of robot ii along the vertical axis; x(3, ii) is the heading of robot ii
% in radians. Easier alternative to dealing with radians is to use
% dxi(:,ii) instead, which is the heading or velocity of robot ii, as a
% vector
% blind_neighbors - matrix tracking the robots in a robot's blind spot
% blind_neighbors not used in the Assignment 
% neighbors - NxN matrix; entry (ii, jj) is 1 if agents ii and jj are
% neighbors; otherwise, entry is 0
% neighbors not used in the Assignment
% N - the number of robots in the swarm
% dxi - the current velocity of the robots (2 x N vector); dxi(1, ii) is
% robot ii's velocity component along the horizontal axis, while dxi(2, ii) 
% is robot ii's velocity component along the vertical axis 
%
% OUTPUTS:
% vel - the resulting velocity of the robots (2 x N vector)
%
% TODO:
% Return the velocity (i.e., heading) that emerges from implementing 
% repulsion, orientation, and attracton interaction rules

%% Authors: Safwan Alam, Musad Haque - 2018
%%%%%%%%%%%%%

% dist(ii, jj) is the distance between robots ii and jj
dist = distances_from_others(x, N); 

% % Random jitter movement <-- REMOVE
% dxi = -1 + 2*rand(2, 10); %<-- REMOVE!!!

% README
% Please run the following commands to run the file sucessfully
% Note that you might need to change the directory_name/file_name since
    % a) swarm.m is renamed
    % b) you download it to a different file folder on your local machine
% 0. Change file name to swarm.m 
% 1. >> run('/Users/susanwangqian/Desktop/sp24_aihw/robotarium/init')
% 2. >> run('/Users/susanwangqian/Desktop/sp24_aihw/robotarium/examples/AI/assignment_3.m')

% repulsion matrix; repulse(i,j)=1 is when j is in i's zone of repulsion
repulsion = zeros(N, N);
% orientation matrix;
orientation = zeros(N, N);
% attraction matrix;
attraction = zeros(N, N); 

for ii = 1:1:N
    for jj = 1:1:N
        if (ii ~= jj)
            if dist(ii, jj) <= rad_rep %zor
                repulsion(ii, jj) = 1;
            elseif dist(ii, jj) > rad_rep && dist(ii,jj) <= rad_ori %zoo
                orientation(ii, jj) = 1;
            elseif dist(ii, jj) > rad_ori && dist(ii,jj) <= rad_att %zoa
                attraction(ii, jj) = 1;
            end
        end
    end
end

dxi_original = dxi;

for ii = 1:1:N
    for jj = 1:1:N
        % Repulsion - move away, -
        if repulsion(ii, jj)
            dxi(1:2, ii) = -x(1:2, jj) + x(1:2, ii) + dxi(1:2, ii);
        end
        % Orientation - align (use original v)
        if orientation(ii, jj) == 1
                dxi(1:2, ii) = dxi_original(1:2, jj) + dxi(1:2, ii);
        end
        % Attraction - move towards, +
        if attraction(ii, jj)
                dxi(1:2, ii) = x(1:2, jj) - x(1:2, ii) + dxi(1:2, ii);
        end
    end

    % Accumulate/aggregate the resulting headings in some fashion, depending on
    % how you implement the three behaviors above.
    % normalize by dividing the magnitude
    if norm(dxi(1:2, ii)) ~= 0
        dxi(1:2, ii) = dxi(1:2, ii)/norm(dxi(1:2, ii));
    end
end

% Return the resulting velocity
vel = dxi;

end

