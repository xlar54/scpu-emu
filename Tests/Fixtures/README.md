# Test fixtures

## traces/

Reference execution traces for differential testing, in the format
`Tools/trace_compare` reads:

```
PC:AAAA A:XX X:XX Y:XX S:XX P:XX CYC:NNNN
```

The main use is milestone 2: in emulation mode the 65816 core must agree with
the 6502 core instruction for instruction. Traces captured from VICE are the
other oracle.

Empty for now — nothing here is yet generated from real hardware or from VICE.
