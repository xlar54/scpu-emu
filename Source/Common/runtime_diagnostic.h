/*
   SCPU-EMU - post-handoff runtime diagnostic arithmetic

   Kept free of Circle dependencies so the counter deltas written by the
   bare-metal reset path can be regression-tested by the host suite.
*/
#ifndef _scpu_runtime_diagnostic_h
#define _scpu_runtime_diagnostic_h

#include "types.h"

struct SCPURuntimeSample
{
	bool handoff;
	u64 hostCycles;
	u64 emuCycles;
	u64 ramWrites;
	u64 acceptedWrites;
	u64 skippedWrites;
	u64 postedWaitCycles;
	u64 slowCycles;
	u64 iecThrottledCycles;
	u64 transfers;
	u64 serialTransfers;
	u64 idlePrimes;
	u32 maxBandUS;
	u32 missedBandDeadlines;
};

struct SCPURuntimeResult
{
	bool valid;
	u64 hostCycles;
	u64 hostUS;
	u64 emuCycles;
	u64 achievedKHz;
	u64 ramWrites;
	u64 acceptedWrites;
	u64 skippedWrites;
	u64 postedWaitCycles;
	u64 slowCycles;
	u64 iecThrottledCycles;
	u64 transfers;
	u64 serialTransfers;
	u64 idlePrimes;
	u32 maxBandUS;
	u32 missedBandDeadlines;
};

inline SCPURuntimeResult scpuRuntimeDelta( const SCPURuntimeSample &start,
	                                       const SCPURuntimeSample &end,
	                                       u32 hostCyclesPerSecond )
{
	SCPURuntimeResult r = {};
	if ( !start.handoff || !end.handoff || !hostCyclesPerSecond
	     || end.hostCycles <= start.hostCycles )
		return r;

	r.valid = true;
	r.hostCycles = end.hostCycles - start.hostCycles;
	r.hostUS = ( r.hostCycles / hostCyclesPerSecond ) * 1000000ull
	         + ( ( r.hostCycles % hostCyclesPerSecond ) * 1000000ull )
	           / hostCyclesPerSecond;
	r.emuCycles = end.emuCycles - start.emuCycles;
	r.achievedKHz = r.hostUS ? r.emuCycles * 1000ull / r.hostUS : 0;
	r.ramWrites = end.ramWrites - start.ramWrites;
	r.acceptedWrites = end.acceptedWrites - start.acceptedWrites;
	r.skippedWrites = end.skippedWrites - start.skippedWrites;
	r.postedWaitCycles = end.postedWaitCycles - start.postedWaitCycles;
	r.slowCycles = end.slowCycles - start.slowCycles;
	r.iecThrottledCycles = end.iecThrottledCycles - start.iecThrottledCycles;
	r.transfers = end.transfers - start.transfers;
	r.serialTransfers = end.serialTransfers - start.serialTransfers;
	r.idlePrimes = end.idlePrimes - start.idlePrimes;
	r.maxBandUS = end.maxBandUS;
	r.missedBandDeadlines = end.missedBandDeadlines
	                        - start.missedBandDeadlines;
	return r;
}

#endif
