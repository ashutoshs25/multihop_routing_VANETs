close all; clear;

% Define the values for the node density - the average number of vehicles per km 

avg_vehicles_per_m = [150 200 250 300 350 400 450 500 550 600 650 700];%(500/4)/(10^3);

% Define the protocol names and their parameter configurations
protocols = {'IFP', 'SDB', 'SPB'};
configurations = {
    struct('k', 5, 'alpha', 15), struct('k', 5, 'alpha', 35), struct('k', 35, 'alpha', 10);
    struct('max_wt', 20), struct('max_wt', 30), struct('max_wt', 40);
    struct('Ns', 512), struct('Ns', 1024), struct('Ns', 2048)
};

% Initialize arrays to store the results
num_protocols = numel(protocols);
num_configs = size(configurations, 2);
pdr_results = zeros(length(avg_vehicles_per_m), num_protocols, num_configs, 100);
latency_results = zeros(length(avg_vehicles_per_m), num_protocols, num_configs, 100);

% Load data from files
for sim_idx = 1:100
    
    % Load PDR data
    PDR = load(strcat('data_nakagami_5dB_5dB/PDR/output_PDR_',num2str(sim_idx),'.mat')).pdr_results;
    pdr_results(:, :, :,sim_idx) = PDR;
    
    % Load latency data
    latency = load(strcat('data_nakagami_5dB_5dB/latency/output_latency_',num2str(sim_idx),'.mat')).latency_results;
    latency_results(:, :, :,sim_idx) = latency;
    
end



pdr_mean = mean(pdr_results, 4);
latency_mean = nanmean(latency_results, 4);

% Plotting
for protocol_idx = 1:num_protocols
    protocol = protocols{protocol_idx};
    
    figure;
    
    
    
    hold on;
    
    
    for config_idx = 1:num_configs
        config = configurations{protocol_idx, config_idx};
        
        % Extract mean data for the current protocol and configuration
        pdr = pdr_mean(:, protocol_idx, config_idx);
        
        % Plot PDR
        plot(avg_vehicles_per_m, pdr);
    end
    
    xlabel('Number of nodes in 4 km long road strip');
    ylabel('PDR');
    title(sprintf('Protocol: %s - PDR', protocol));
    legend('Location', 'best');
    hold off;
    
    figure;
    hold on;
    
    for config_idx = 1:num_configs
        config = configurations{protocol_idx, config_idx};
        
        % Extract mean data for the current protocol and configuration
        latency = latency_mean(:, protocol_idx, config_idx);
        
        % Plot latency
        plot(avg_vehicles_per_m, latency*1000);
    end

    xlabel('Number of nodes in 4 km long road strip');
    ylabel('Latency');
    title(sprintf('Protocol: %s - Latency', protocol));
    legend('Location', 'best');
    hold off;


end
    
 




    
  
