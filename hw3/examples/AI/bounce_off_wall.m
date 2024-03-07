function final_velocity = bounce_off_wall(dxi, x, N)
% FILE: bounce_off_wall.m makes the robots stay within the boundaries of
% robotarium
%
% DESCRIPTION: Manipulate velocities to bounce off walls by adding a push
% velocity either up, down, left, or right -- depending on position with 
% respect to the wall. 
% 
% INPUTS: 
% x - matrix containing the pose of all the robots
% N - the number of robots in the swarm
% dxi - the original velocity of all the robots
%
% OUTPUTS:
% The final velocity of the robots.
% 
%
% TODO:
% None

%% Authors: Safwan Alam, Musad Haque - 2018 
%%%%%%%%%%%%%

safety_distance = 0.10;

for i = 1:N
    if x(1,i) + safety_distance >= 1.60
        dxi(1:2, i) = [-1;0] + dxi(1:2, i);
        if norm(dxi(1:2, i)) ~= 0
            dxi(1:2, i) = dxi(1:2, i)/norm(dxi(1:2, i));
        end
    elseif x(1,i) - safety_distance <= -1.60
        dxi(1:2, i) = [1;0] + dxi(1:2, i);
        if norm(dxi(1:2, i)) ~= 0
            dxi(1:2, i) = dxi(1:2, i)/norm(dxi(1:2, i));
        end
    end
    if x(2,i) + safety_distance >= 1.00
        dxi(1:2, i) = [0;-1] + dxi(1:2, i);
        if norm(dxi(1:2, i)) ~= 0
            dxi(1:2, i) = dxi(1:2, i)/norm(dxi(1:2, i));
        end
    elseif x(2,i) - safety_distance <= -1.00
        dxi(1:2, i) = [0;1] + dxi(1:2, i);
        if norm(dxi(1:2, i)) ~= 0
            dxi(1:2, i) = dxi(1:2, i)/norm(dxi(1:2, i));
        end
    end
    
end

final_velocity = dxi;

end


