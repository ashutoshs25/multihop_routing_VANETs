function PL = LoS_pathloss(d,fc)


PL = 32.4 + 20*log10(d) + 20*log10(fc);


end
