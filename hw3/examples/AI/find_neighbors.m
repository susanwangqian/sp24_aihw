function [neighbors] = find_neighbors(communication_model, N, x, topo_dist, radius_metric)
% FILE: find_neighbors.m determines which robots are within comms
%
% DESCRIPTION: Applies the selected communication model to each agent in
% order to identify neighbors
%
% INPUTS:
% 1. communication_model - a string that indicates which comms model to use
% 2. N - the number of robots in the swarm
% 3. x - matrix containing the pose of all the robots
% 4. topo_dist - the nearest neighbor number used in the topological model
% 5. radius_metric - the comms radius used in the metric model
%
% OUTPUTS:
% An N x N matrix representing which robots are neighbors with one another 
% 
% TODO:
% Move visual parameters out of here

%% Authors: Safwan Alam, Musad Haque - 2018 
%%%%%%%%%%%%%

dist = distances_from_others(x, N);

%Initializing values for communication models
neighbors = zeros(N, N);
visual_max_error = 0.9;
angle_of_occulsion = 0.1;

switch communication_model
    case 'M'
        %Metric Model
        for ii = 1 : N
            for jj = 1 : N
                if dist(ii, jj) == 0
                    neighbors(ii,jj) = 0;
                elseif dist(ii, jj) <= radius_metric
                    neighbors(ii,jj) = 1;
                end
            end
        end
    case 'T'
        %Topological
        for ii = 1:N
            [~, sorted_d_indicies] = sort(dist(ii,:));
            for jj = 1:(topo_dist + 1)%cant be my own neighbor
                if ii ~= sorted_d_indicies(jj)
                    neighbors(ii, sorted_d_indicies(jj)) = 1;
                end
            end
        end
    case 'V'
        for ii = 1:N
            for jj = 1:N
                if(ii ~= jj)
                    angle = atan2((x(2,jj) - x(2,ii)),(x(1,jj) - x(1,ii)));
                    if abs(x(3,ii)-angle) <= visual_max_error
                        neighbors(ii,jj) = 1;
                    end
                end
            end
        end
        for ii = 1:N
            for jj = 1:N
                if neighbors(ii,jj) == 1
                    for k = 1:N
                        if ii ~= jj && ii ~= k && k ~= jj
                            angle_between_IJ = atan2((x(2,jj) - x(2,ii))...
                                ,(x(1,jj) - x(1,ii)));
                            angle_between_IK = atan2((x(2,k) - x(2,ii))...
                                ,(x(1,k) - x(1,ii)));
                            if abs(angle_between_IJ - angle_between_IK) ...
                                    <= angle_of_occulsion/dist(ii,jj)
                                if dist(ii,k) > dist(ii,jj)
                                    neighbors(ii,k) = 0;
                                elseif dist(ii,jj) > dist(ii,k)
                                    neighbors(ii,jj) = 0;
                                end
                            end
                        end
                    end
                end
            end
        end
    otherwise
        
end

end


