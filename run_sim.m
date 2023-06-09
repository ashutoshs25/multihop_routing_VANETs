
% Use this with a random aID when running 1 simulation on local PC
aID = '4';



% Uncomment the following lines for running on HPC

%aID = getenv('SLURM_ARRAY_TASK_ID');
% if(isempty(aID))
%   warning('aID is empty. Replacing it with 1.');
%   aID = '1';
% end

rng(str2double(aID),'twister');

% Only needed when running on NYU HPC 

% The path here should correspond to the code directory in your HPC login node
addpath('/scratch/as12738/Project');



sim_time = 100; % seconds
message_rate = 1; % messages per second

% simulation parameters
slot_time = 40 * 10^-6; % 40 us
SIFS = 10 * 10^-6; % 10 us
DIFS = SIFS + 2*slot_time;
num_slots = sim_time/slot_time; % total slots

R = 300; % transmission range (metres)
active_TXs = [];
message_size = 50; % bytes
data_rate = 6*10^6; %bps

% wireless communication parameters
B = 10; % MHz
NF = 9; % dB
fc = 5.9; % GHz
P_tx = 14; %dBm
G_tx = 0;  %dB
G_rx = 0;  %dB
SNR_thresh = 8; %dB

% protocol = ['IFP','SDB','SPB'];
% 
% protocol = 'SDB';

% IFP broadcast algorithm parameters
CW_base = 2;
alpha = 10;
k = 35;

% Parameter for SDB - simple delay based protocol
max_wt = (40*10^-3)/(slot_time);

% Parameters for p-persistent stochastic protocol (SPB) 
N_s = 1024; p=0.8;

% retransmission 
timeout = 20*10^-3; %ms

% Define the length of the lane in km
lane_length_m = 4*10^3;

% Define the values for the node density - the average number of vehicles per km 

avg_vehicles_per_m = [150 200 250 300 350 400 450 500 550 600 650 700].*(1/(4*10^3));%(500/4)/(10^3);


% Testing three different multi-hop routing protocol
protocols = {'IFP', 'SDB', 'SPB'};
protocols = {'IFP'};
configurations = {
    struct('k', 5, 'alpha', 15), struct('k', 5, 'alpha', 35), struct('k', 35, 'alpha', 10);
    struct('max_wt', 20), struct('max_wt', 30), struct('max_wt', 40);
    struct('Ns', 512), struct('Ns', 1024), struct('Ns', 2048)
};

configurations = {struct('k',5,'alpha',15)};

num_protocols = numel(protocols);
num_configs = size(configurations, 2);
pdr_results = zeros(length(avg_vehicles_per_m), num_protocols, num_configs);
latency_results = zeros(length(avg_vehicles_per_m), num_protocols, num_configs);


for exp=1:length(avg_vehicles_per_m) % run an experiment for each node density value
    
    % Generate vehicle locations
    
    % Calculate the expected number of vehicles on the lane
    expected_num_vehicles = avg_vehicles_per_m(exp) * lane_length_m;

    % Generate the Poisson distributed random variable
    num_vehicles = poissrnd(expected_num_vehicles);

    % Generate the locations of the vehicles using a uniform distribution

    vehicle_locations = sort(rand(num_vehicles,1)*lane_length_m);  
    
    
    % Generate traffic
    
    message_times = [];
    vehicle_ids = [];
    
    for i=1:num_vehicles

        first_message_time = exprnd(1/message_rate);
        t = first_message_time;

        while t<sim_time

            message_times = [message_times t];
            vehicle_ids = [vehicle_ids i];
            t = t + exprnd(1/message_rate);

        end
    end
    
    % Sort the message_times and vehicle_ids arrays based on message_times values in increasing order
    [message_times_sorted, sort_idx] = sort(message_times);
    vehicle_ids_sorted = vehicle_ids(sort_idx);

    % Find the indices of all messages generated by vehicle with ID 1
    vehicle1_msg_idx = find(vehicle_ids_sorted == 1);

    % Extract the message times for vehicle 1
    vehicle1_msg_times = message_times_sorted(vehicle1_msg_idx);
    
    
    

    % Loop through the protocols and configurations
    for protocol_idx = 1:num_protocols
        protocol = protocols{protocol_idx};
        parameter_configs = configurations(protocol_idx, :);

       % fprintf('Running simulation for protocol: %s\n', protocol);

        
        for config_idx = 1:num_configs
            config = parameter_configs{config_idx};

           % fprintf('Running simulation for configuration %d: ', config_idx);

            if strcmp(protocol, 'IFP')
               % fprintf('k = %d, alpha = %d\n', config.k, config.alpha);
                
                [PDR, latency] = vanet_broadcast_sim(num_slots, slot_time, R, protocol, vehicle_locations, num_vehicles, vehicle1_msg_times, P_tx,fc,G_tx,G_rx,NF,B,SNR_thresh, timeout, CW_base, config.alpha, config.k, max_wt, N_s, p);
           
            elseif strcmp(protocol, 'SDB')
               % fprintf('max_wt = %d\n', config.max_wt);
               
                max_wt = (config.max_wt*10^-3)/slot_time;
                
                [PDR, latency] = vanet_broadcast_sim(num_slots, slot_time, R, protocol, vehicle_locations, num_vehicles, vehicle1_msg_times, P_tx,fc,G_tx,G_rx,NF,B,SNR_thresh, timeout, CW_base, alpha, k, max_wt, N_s, p);
            
            elseif strcmp(protocol, 'SPB')
               % fprintf('Ns = %d\n', config.Ns);
                
                [PDR, latency] = vanet_broadcast_sim(num_slots, slot_time, R, protocol, vehicle_locations, num_vehicles, vehicle1_msg_times, P_tx,fc,G_tx,G_rx,NF,B,SNR_thresh, timeout, CW_base, alpha, k, max_wt, config.Ns, p);
            else
                fprintf('Unknown protocol\n');
            end
        
            % run VANET broadcast simulation
            

            pdr_results(exp,protocol_idx, config_idx) = PDR;
            latency_results(exp,protocol_idx, config_idx) = latency;
        
           % fprintf('Simulation for configuration %d completed.\n\n', config_idx);
        end

        %fprintf('Simulation for protocol: %s completed.\n\n', protocol);
    end

    
    
end


savefolder = strcat('data/PDR');
if ~exist(savefolder, 'dir')
       mkdir(savefolder)
end
save(strcat('data/PDR/','output_PDR','_',num2str(aID),'.mat'),'pdr_results');

savefolder = strcat('data/latency');
if ~exist(savefolder, 'dir')
       mkdir(savefolder)
end
save(strcat('data/latency/','output_latency','_',num2str(aID),'.mat'),'latency_results');




















