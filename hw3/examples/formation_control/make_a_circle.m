%Initializing the agents to random positions with barrier certificates 
%and data plotting.  This script shows how to initialize robots to a
%particular point
%Paul Glotfelter 
%3/24/2016

% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = 16; 


% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
r = rb.set_number_of_agents(N).set_save_data(false).build();

% Initialize x so that we don't run into problems later.  This isn't always
% necessary
x = r.get_poses();
r.step();

% Create a barrier certificate so that the robots don't collide
si_barrier_certificate = create_si_barrier_certificate('SafetyRadius', 0.06);
si_to_uni_dynamics = create_si_to_uni_mapping2();
        
%Get randomized initial conditions in the robotarium arena
initial_conditions = [1, cos(pi/8) ,sqrt(2)/2, cos(3*pi/8), 0, cos(5*pi/8),  -sqrt(2)/2, cos(7*pi/8), -1, cos(9*pi/8), -sqrt(2)/2, cos(11*pi/8), 0, cos(13*pi/8), sqrt(2)/2, cos(15*pi/8), ;
                      0, sin(pi/8),  sqrt(2)/2, sin(3*pi/8), 1, sin(5*pi/8), sqrt(2)/2, sin(7*pi/8), 0, sin(9*pi/8), -sqrt(2)/2, sin(11*pi/8), -1, sin(13*pi/8), -sqrt(2)/2, sin(15*pi/8), ;
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

% We'll make the rotation error huge so that the initialization checker
% doesn't care about it
args = {'PositionError', 0.01, 'RotationError', 50};
init_checker = create_is_initialized(args{:});
controller = create_si_position_controller();

while(~init_checker(x, initial_conditions))

    x = r.get_poses();
    dxi = controller(x(1:2, :), initial_conditions(1:2, :));
    dxi = si_barrier_certificate(dxi, x(1:2, :));      
    dxu = si_to_uni_dynamics(dxi, x);

    r.set_velocities(1:N, dxu);
    r.step();   
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();

