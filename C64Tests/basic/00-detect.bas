10 print chr$(147);"scpu detection and status"
20 d=peek(53436):if (d and 128)<>0 then print "no supercpu detected":end
30 print "scpu detected"
40 v=peek(53424):print "version/mode $d0b0:";v
50 r=peek(53426):print "$d0b2 status:";r;" (128=regs,64=slow)"
60 s=peek(53429):print "$d0b5 switches:";s;" (128=jiffy,64=normal)"
70 m=peek(53432):print "$d0b8 speed:";m;" (128=soft,64=master)"
80 print:print "pass"
