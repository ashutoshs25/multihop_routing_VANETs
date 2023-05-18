function fading_gain = nakagami_gain()
  
% returns a randomly generated nakagami fading channel gain
%K = 7;


m=4;

g = gamrnd(m,1/m);

fading_gain = 10*log10(g);  % channel gain in dB
end     