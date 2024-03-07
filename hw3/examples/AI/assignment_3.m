%%%%% Get Robotarium object used to communicate with the robots/simulator
% Do not modify!!!
rb = RobotariumBuilder();
%%%%%

%%%%%% Parameters
% Do not modify!!!
%%%%%

% Total iterations in an episode
total_iterations = 2500;

% Time step
time_step = 1;

% Swarm size: number of agents
N = 10;

% Radius of each robot is 0.08 repulsion-orientation-attraction
rad_rep = 0.16;
rad_ori = 0.24;
rad_att = 4.00;

%%%% Temeperature stuff for the 22-Feb-2024 Lecture. Ignore for Assignment.
% Initial estimates
temperature_est = 100*rand(N, 1);
% Store for temperature estimates
temperature_est_history = [];
% Nearest neighbors, if using Topological model to communicate
topo_dist = 4;
% Blind spot, if using Visual model to communicate
half_sense = pi*105/180;
% The Compromised Agent is chosen at random.
bad_robot = randi(N);
%%%%%

%%%% Robotarium setup
% Do not modify!!!
r = rb.set_number_of_agents(N).set_save_data(false).build();
x = r.get_poses();
r.step();
si_barrier_certificate = create_si_barrier_certificate('SafetyRadius',0.001);
si_to_uni_dynamics = create_si_to_uni_mapping2();
args = {'PositionError', 0.01, 'RotationError', 50};
init_checker = create_is_initialized(args{:});
controller = create_si_position_controller();
dxi = zeros(2,N); % The 2D velocity vector of all N robots
%%%%%

%%%%% Simulation
% Modify, if absolutely needed
%%%%%

% For each iteration in time:
for t = 1:time_step:total_iterations

    % Get the latest pose information
    x = r.get_poses();
    
    % Get neighbors using the metric model (range set to the rad_att)
    [neighbors] = find_neighbors('M', N, x, topo_dist, rad_att);

    % Check everyone's blind spots
    [blind_neighbors] = check_blind_spot(x, N, half_sense); %<-- Ignore for Assignment
    
    % Update 2D velocity vectors (headings) so that agents swarm
    [dxi] = swarm(rad_rep, rad_ori, rad_att, x, N, dxi);

    % Update 2D velocity vectors (headings) in order to stay in the arena
    dxi = bounce_off_wall(dxi, x, N);

    %%%%% Temeperature stuff for the 22-Feb-2024 Lecture.
    % How much should the sensor estimate of each agent change?
    delta_temperature_est = estimate(temperature_est, neighbors, N, bad_robot); %<-- Ignore for Assignment

    % Update sensor estimate by incorporating the amount of change
    temperature_est = temperature_est + (time_step * delta_temperature_est); %<-- Ignore for Assignment

    % Bad robot does its own thing
    temperature_est(bad_robot, 1) = malicious_estimate; %<-- Ignore for Assignment

    % Store everyone's history of temperature estimates for plotting purposes
    temperature_est_history = [temperature_est_history, temperature_est]; %<-- Ignore for Assignment
    %%%%%

    %%%%% Robotarium uses dxi to move the agents
    % Do not modify!!!
    dxi = si_barrier_certificate(dxi, x(1:2, :));
    dxu = si_to_uni_dynamics(dxi, x);
    r.set_velocities(1:N, dxu);
    r.step();
    %%%%%

    % Save a frame every 100 iterations
    if (~mod(t,100))
        saveas(gcf,['t',num2str(t)],'png');
    end

end

%%%%% Plotting
% Do not modify!!!
r.call_at_scripts_end();
figure; % Plot the sensor estimates for each agent over time
for ii=1:1:N
    plot(temperature_est_history(ii, 1:1:total_iterations), 'LineWidth', 2);
    hold on;
end
axis([-100 total_iterations 0 100]);
xlabel('time iteration');
ylabel('temperature estimates');
saveas(gcf,'temperature_estimation','png');
%%%%%


