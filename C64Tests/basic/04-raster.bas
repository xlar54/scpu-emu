10 print chr$(147);"vic raster/jiffy test"
20 lo=999:hi=0:fr=0:la=0:ti$="000000"
30 h1=peek(53265)and128
31 rl=peek(53266)
32 h2=peek(53265)and128
33 if h1<>h2 then goto 30
34 r=rl+h1*2
35 if r<la then fr=fr+1
36 la=r
40 if r<lo then lo=r
50 if r>hi then hi=r
60 if ti<60 then goto 30
70 print chr$(19);"observed raster:";lo;"to";hi;"   "
80 print "frames in one second:";fr;"   "
90 print "run/stop exits; display must update"
100 lo=999:hi=0:fr=0:la=0:ti$="000000":goto 30
