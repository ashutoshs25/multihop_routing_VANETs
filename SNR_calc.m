
function [SNR,P_rx] = SNR_calc(P_tx,d,fc,G_tx,G_rx,NF,B)


PL = LoS_pathloss(d,fc);

fading = rayleigh;

P_noise = -174 + NF + 10*log10(B*10^6); %+ fading;


P_rx = P_tx + G_tx + G_rx - PL + fading;
SNR = P_rx - P_noise;


end
