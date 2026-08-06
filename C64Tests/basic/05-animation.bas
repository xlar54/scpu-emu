10 print chr$(147);"supercpu 2.04 startup animation"
20 if (peek(53436)and128)<>0 then print "no supercpu detected":end
30 print "refreshing animation from rom..."
40 for i=0 to 4:read b:poke 49152+i,b:next
50 print "basic will restart when it finishes"
60 sys 49152
70 end
100 data 120,92,224,1,248
