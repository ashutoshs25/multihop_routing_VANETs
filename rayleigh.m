function fading_gain = rayleigh()

% returns a randomly generated rayleigh fading channel gain


h=(1/sqrt(2))*(randn(1)+1i*randn(1));  % complex gaussian
 
g = norm(h).^2;             

fading_gain = 10*log10(g);  % channel gain in dB
end
