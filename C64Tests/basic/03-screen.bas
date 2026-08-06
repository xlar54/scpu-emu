10 print chr$(147);"screen/color mirror test"
20 if (peek(53436) and 128)<>0 then print "warning: no supercpu"
30 poke 53280,6:poke 53281,0
40 for y=0 to 15:for x=0 to 15
50 p=1024+80+y*40+x:c=(x+y*16)and255
60 poke p,c:poke 55296+80+y*40+x,(x+y)and15
70 next:next
80 print chr$(19);"screen/color mirror test"
90 print:print:print:print:print:print:print:print:print:print:print:print:print:print:print:print:print
100 print "pass if field is stable and coloured"
