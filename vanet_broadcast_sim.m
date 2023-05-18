function [PDR, latency] = vanet_broadcast_sim(num_slots, slot_time, R, protocol, vehicle_locations, num_vehicles, vehicle1_msg_times, P_tx,fc,G_tx,G_rx,NF,B,SNR_thresh, timeout, CW_base, alpha, k, max_wt, N_s, p)


    % 1 - message was ACKed, 0 - not ACKed 
    vehicle1_ack_status = zeros(1,length(vehicle1_msg_times));

    % tracking current transmission ID
    next_Tx_idx = 1;

    % transmission and retransmission times
    vehicle1_Tx_time = zeros(1,length(vehicle1_msg_times));
    vehicle1_reTx_time = zeros(1,length(vehicle1_msg_times));

    % tracking broadcast times, packet reception, latency and CW 
    broadcast_times = zeros(1,length(vehicle_locations));
    pkt_received = zeros(1,length(vehicle_locations));
    curr_pkt_id = zeros(1,length(vehicle_locations));
    reception_latency = zeros(length(vehicle1_msg_times),length(vehicle_locations));
    reception_count = zeros(length(vehicle1_msg_times),length(vehicle_locations));
    CW_max_values = zeros(1,length(vehicle_locations));
    
    
    forwarding_times = [];

    retx=0;

    
    for i=1:num_slots
    
        % Single source - vehicle 1 (start of the road)


        % First, we deal with Vehicle 1 (Leader)


        % TRANSMISSION
        % if last transmission of vehcile 1 has been acked
        if next_Tx_idx == 1 || vehicle1_ack_status(next_Tx_idx-1) == 1

            if next_Tx_idx > length(vehicle1_msg_times)
            else
                if vehicle1_msg_times(next_Tx_idx) <= i*slot_time && vehicle1_Tx_time(next_Tx_idx) == 0

                    vehicle1_Tx_time(next_Tx_idx) = i*slot_time;
                    
                     broadcast_times = zeros(1,length(vehicle_locations));
                     pkt_received = zeros(1,length(vehicle_locations));
                     curr_pkt_id = zeros(1,length(vehicle_locations)) + next_Tx_idx-1;
                     CW_max_values = NaN(1,length(vehicle_locations));

                    leader = 1;  % vehicle 1 is the leader at the beginning
                else

                    % do nothing, new message has not arrived yet
                end
            end
        else
            % wait for ACK before starting next transmission
        end

        % Is any other vehicle broadcasting the source packet in this slot? 
        idx = find(broadcast_times == i);
        
        % implement p-persistent broadcast 
        if strcmpi(protocol,'SPB')

            idx_new=[];
            for k = 1:length(idx)
                    % Randomly decide whether to remove the element
                    if rand < p
                    % Remove the element
                        idx_new = [idx_new, idx(k)];
                    end
            end

            idx = idx_new;

        end

        if length(idx)>1   % YES

            for k=1:length(idx)    % handle broadcast collisions 

                CW_max = CW_max_values(idx(k));

                % choose new broadcast time  

                if strcmpi(protocol,'IFP')
                    %IFP
                    broadcast_times(idx(k)) = i + randi(ceil(CW_max)+1) - 1;
                elseif strcmpi(protocol,'SDB')
                    %SDB
                    broadcast_times(idx(k)) = ceil(i + (max_wt)*(1 - CW_max/R)); 
                else
                    %SPB
                    broadcast_times(idx(k)) = ceil(i + (N_s * (1-CW_max/R)));
                end

            end

            idx =[];  % assume collision when multiple relays broadcast in the same slot
        end


        % ACKING procedure
        if ~isempty(idx)


            % select a new leader  

           old_leader = leader;  

            leader = idx; leader_pkt_id = curr_pkt_id(idx);

            d = norm(vehicle_locations(1,:) - vehicle_locations(idx,:));

            [SNR, P_rx] = SNR_calc(P_tx,d,fc,G_tx,G_rx,NF,B);

            if (d<=R) && (SNR >= SNR_thresh)

                %if next_Tx_idx == 1 || vehicle1_ack_status(next_Tx_idx) == 0

                if leader_pkt_id == next_Tx_idx 

                    vehicle1_ack_status(next_Tx_idx) = 1;  % ACK received

                    next_Tx_idx = min(next_Tx_idx + 1,length(vehicle1_msg_times));
                end
            end
        end

        % Handle ACK timeouts using retransmissions

        if  vehicle1_Tx_time(next_Tx_idx) > 0 && (i*slot_time - vehicle1_Tx_time(next_Tx_idx)) >= timeout && retx==0

            % ACK timeout
            retx=1;
            vehicle1_reTx_time(next_Tx_idx) = i*slot_time;
        end



        % Now we look at the follower vehicles 
        for j=2:num_vehicles


            % if lead vehicle sent out a new packet in this slot
            if vehicle1_Tx_time(next_Tx_idx) == i*slot_time

                d = norm(vehicle_locations(1,:) - vehicle_locations(j,:));

                [SNR, P_rx] = SNR_calc(P_tx,d,fc,G_tx,G_rx,NF,B);

                if (d<=R) && (SNR >= SNR_thresh)

                    pkt_received(j) = 1;
                    curr_pkt_id(j) = next_Tx_idx;
                    
                    % calculate latency of reception 
                    reception_latency(curr_pkt_id(j),j) = i*slot_time - vehicle1_msg_times(curr_pkt_id(j));
                    reception_count(curr_pkt_id(j),j) = 1;

                    if strcmpi(protocol,'IFP')
                        % calculate CW_max and random broadcast time (IFP)
                        CW_max = k*(R/d)*(CW_base^((SNR - SNR_thresh)/alpha));
                        CW_max_values(j) = CW_max;
                        broadcast_times(j) = i + randi(ceil(CW_max)+1) - 1;

                    elseif strcmpi(protocol,'SDB')
                        % Simple delay-based protocol (SDB)
                        broadcast_times(j) = ceil(i + (max_wt)*(1 - d/R));    
                        CW_max_values(j) = d;
                    else
                        % Simple probabilistic protocol (SPB)
                        broadcast_times(j) = ceil(i + (N_s * (1-d/R)));
                        CW_max_values(j) = d; 
                    end

                end
            else
            end


            % If a forwarder broadcasted the source packet in this slot

            if ~isempty(idx)

                if j == idx
    %                 broadcast_times(j) = -1;           
    %                 pkt_received(j) = 0;

                elseif j == old_leader  % 

                    broadcast_times(j) = -1;
                    pkt_received(j) = 0;

                else

                    d = norm(vehicle_locations(idx,:) - vehicle_locations(j,:));
                    [SNR, P_rx] = SNR_calc(P_tx,d,fc,G_tx,G_rx,NF,B);
                    
                    if curr_pkt_id(j) < leader_pkt_id
                        
                        if (d<=R) && (SNR >= SNR_thresh)
                                                        
                            d1 = norm(vehicle_locations(1,:) - vehicle_locations(j,:));
                            d2 = norm(vehicle_locations(1,:) - vehicle_locations(idx,:));
                            
                            if d1 < d2
                                broadcast_times(j) = -1;
                                pkt_received(j) = 0;
                                curr_pkt_id(j) = leader_pkt_id;
                                reception_latency(curr_pkt_id(j),j) = i*slot_time - vehicle1_msg_times(curr_pkt_id(j));
                                reception_count(curr_pkt_id(j),j) = 1;
                            else
                                pkt_received(j) = 1;
                                curr_pkt_id(j) = leader_pkt_id;
                                % calculate latency of reception 
                                reception_latency(curr_pkt_id(j),j) = i*slot_time - vehicle1_msg_times(curr_pkt_id(j));
                                reception_count(curr_pkt_id(j),j) = 1;

                                if strcmpi(protocol,'IFP')
                                    % calculate CW_max and random broadcast time (IFP)
                                    CW_max = k*(R/d)*(CW_base^((SNR - SNR_thresh)/alpha));
                                    CW_max_values(j) = CW_max;
                                    broadcast_times(j) = i + randi(ceil(CW_max)+1) - 1;
                                elseif strcmpi(protocol,'SDB')
                                    % Simple delay-based protocol (SDB)
                                    broadcast_times(j) = ceil(i + (max_wt)*(1 - d/R));    
                                    CW_max_values(j) = d;
                                else
                                    % Simple probabilistic protocol (SDB)
                                    broadcast_times(j) = ceil(i + (N_s * (1-d/R)));
                                    CW_max_values(j) = d; 
                                end    
                            end
                        end
                    else
                        if pkt_received(j) == 1 
                            
                            %if SNR >= SNR_thresh
                                
                                broadcast_times(j) = -1;
                                pkt_received(j) = 0;
                           % end
                        end
                        
                    end

                end

            end


            if vehicle1_reTx_time(next_Tx_idx) == i*slot_time

                d = norm(vehicle_locations(1,:) - vehicle_locations(j,:));

                [SNR, P_rx] = SNR_calc(P_tx,d,fc,G_tx,G_rx,NF,B);

                if (d<=R) &&  (SNR >= SNR_thresh)

                    if broadcast_times(j) == -1 && vehicle1_ack_status(next_Tx_idx) == 0

                        vehicle1_ack_status(next_Tx_idx) = 1;

                        retx = 0;

                        next_Tx_idx = min(next_Tx_idx + 1,length(vehicle1_msg_times));

                    end

                end

            end

        end
       % curr_time=i*slot_time
    end

    reception_count(:,1) = [];
    reception_latency(:,1) = [];

    latency_E2E = NaN(1, length(vehicle1_msg_times));

    for m = 1:length(vehicle1_msg_times)

        if reception_count(m,end) == 1
            latency_E2E(m) = reception_latency(m,end);
        else
            latency_E2E(m) = 0.1;   % timeout penalty (100 ms)
        end

    end

    PDR = mean(reception_count,'all');
    latency = nanmean(latency_E2E);

    
    
end