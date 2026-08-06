10 print chr$(147);"scpu normal/turbo benchmark"
20 if (peek(53436) and 128)<>0 then print "no supercpu detected":end
30 n=12000:print "iterations:";n
40 poke 53370,0:gosub 200:sn=t
50 poke 53371,0:gosub 200:ft=t
60 poke 53370,0
70 print "normal jiffies:";sn
80 print "turbo  jiffies:";ft
90 if ft>=sn then print "fail: turbo not faster":goto 120
100 print "speed ratio:";int(sn*10/ft)/10
110 print "pass (physical switch must be turbo)"
120 poke 53371,0:end
200 ti$="000000":a=0
210 for i=1 to n:a=a+i-int(a/997)*997:next
220 t=ti:return
