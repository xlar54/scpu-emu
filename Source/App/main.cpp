/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   Circle bare-metal entry point.

   Structure follows the RAD framework: a Circle kernel object owns the screen,
   timer, interrupt and SD-card subsystems, brings up the GPIO and the ARM cycle
   counter, and then hands off to the accelerator. Once running we hold DMA and
   drive the C64 bus against a hard cycle budget, so almost nothing of Circle is
   used after start-up.

   Built on the RAD Expansion Unit framework
   Copyright (c) 2022, 2023 Carsten Dachsbacher <frenetic@dachsbacher.de>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <circle/startup.h>
#include <circle/bcm2835.h>
#include <circle/memio.h>
#include <circle/memory.h>
#include <circle/koptions.h>
#include <circle/devicenameservice.h>
#include <circle/screen.h>
#include <circle/interrupt.h>
#include <circle/timer.h>
#include <circle/logger.h>
#include <circle/sched/scheduler.h>
#include <circle/types.h>
#include <circle/util.h>
#include <SDCard/emmc.h>
#include <fatfs/ff.h>

#include "../Bus/RAD/lowlevel_arm64.h"
#include "../Bus/RAD/gpio_defs.h"
#include "../Bus/RAD/c128_refresh.h"
#include "../Common/helpers.h"
#include "../Common/version.h"
#include "boot.h"

CLogger *logger;

class CSCPUKernel
{
public:
	CSCPUKernel( void )
		: m_CPUThrottle( CPUSpeedMaximum ),
		  m_Screen( m_Options.GetWidth(), m_Options.GetHeight() ),
		  m_Timer( &m_Interrupt ),
		  m_Logger( 5, &m_Timer ),
		  m_EMMC( &m_Interrupt, &m_Timer, 0 ),
		  m_C128Refresh( &m_Memory )
	{
	}

	boolean Initialize( void )
	{
		logger = &m_Logger;
		STANDARD_SETUP_TIMER_INTERRUPT_CYCLECOUNTER_GPIO
		// Circle requires multi-core support to be initialized last. Core 3 then
		// waits dormant until a physical C128 has actually been acquired.
		if ( bOK ) bOK = m_C128Refresh.Initialize();
		return bOK;
	}

	void Run( void );

private:
	CMemorySystem		m_Memory;
	CKernelOptions		m_Options;
	CDeviceNameService	m_DeviceNameService;
	CCPUThrottle		m_CPUThrottle;
	CScreenDevice		m_Screen;
	CInterruptSystem	m_Interrupt;
	CTimer				m_Timer;
	CLogger				m_Logger;
	CScheduler			m_Scheduler;
	CEMMCDevice			m_EMMC;
	CC128RefreshService m_C128Refresh;
};

static FATFS mFileSystem;

extern "C" void radMountFileSystem()
{
	if ( f_mount( &mFileSystem, "SD:", 1 ) != FR_OK )
		logger->Write( "SCPU", LogError, "failed mounting partition 'SD:'" );
}

extern "C" void radUnmountFileSystem()
{
	f_unmount( "SD:" );
}

void CSCPUKernel::Run( void )
{
	gpioInit();
	m_EMMC.Initialize();

	// Run the ARM flat out: every bus access is timed against the cycle
	// counter, and a throttled clock would move all the timing constants.
	m_CPUThrottle.SetSpeed( CPUSpeedMaximum );

	// This deliberately precedes even the SD-card mount. If startup fails, the
	// identity and licensing context are still the first project text visible
	// on the HDMI console.
	logger->Write( "SCPU", LogNotice, "SuperCPU Emulator - Build #%s",
	               SCPU_EMULATOR_BUILD );
	logger->Write( "SCPU", LogNotice, "" );
	logger->Write( "SCPU", LogNotice, "Author: Scott Hutter / xlar54" );
	logger->Write( "SCPU", LogNotice, "https://github.com/xlar54/scpu-emu" );
	logger->Write( "SCPU", LogNotice, "" );
	logger->Write( "SCPU", LogNotice,
	               "RAD REU developed by Carsten Dachsbacher" );
	logger->Write( "SCPU", LogNotice, "https://github.com/frntc/RAD" );
	logger->Write( "SCPU", LogNotice, "" );
	logger->Write( "SCPU", LogNotice,
	               "Portions of this project are based on source code from the "
	               "VICE Team (VICE Commodore Emulator) and are distributed "
	               "under the terms of the GNU General Public License (GPL)." );
	logger->Write( "SCPU", LogNotice, "" );
	logger->Write( "SCPU", LogNotice,
	               "CMD is a trademark of Creative Micro Designs, Inc." );
	logger->Write( "SCPU", LogNotice,
	               "Commodore is a trademark of its respective owner(s)." );
	logger->Write( "SCPU", LogNotice, "" );

	radMountFileSystem();

	scpuBootLoadConfig( logger );
	scpuBootRun( logger );

	// scpuBootRun() only returns on a failure it has already logged.
	while ( 1 )
		;
}

int main( void )
{
	CSCPUKernel kernel;

	if ( kernel.Initialize() )
		kernel.Run();

	halt();
	return EXIT_HALT;
}
