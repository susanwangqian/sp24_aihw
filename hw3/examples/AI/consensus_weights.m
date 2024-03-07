% Four agents in the network, Assume K4 graph
N = 4;

% Initial sensor readings
reading = [1000; 100; 100; 100];

% Hiostory of readings for plotting
reading_history = [];

% Let's play with this value
weight = 0.5;

for t = 1:1:300

    % save history of readings
    reading_history = [reading_history, reading];

    % how much should agents update their readings? (initialize to 0) 
    delta_reading = zeros(N, 1);

    % for each agent i in the network
    for ii = 1:1:N
        % for each agent j that is i's neighbor
        for jj = 1:1:N
            if (ii ~= jj)
                % apply consensus protocol
                % this tells an agent how much to change its reading
                delta_reading(ii, 1) = delta_reading(ii, 1) - (reading(ii, 1) - reading(jj, 1));
            end
        end
    end

    % update reading
    reading = reading + (weight * delta_reading);

end

% Plot
figure;
for ii=1:1:N
    plot(reading_history(ii, 1:1:300), 'LineWidth', 2);
    hold on;
end
axis([0 50 100 1000]);
xlabel('time iteration');
ylabel('sensor reading');


