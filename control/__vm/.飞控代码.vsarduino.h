/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Mega 2560 HAL (Apm 2), Platform=avr, Package=apm
*/

#ifndef _VSARDUINO_H_
#define _VSARDUINO_H_
#define __AVR_ATmega2560__
#define ARDUINO 166
#define ARDUINO_MAIN
#define __AVR__
#define __avr__
#define F_CPU 16000000L
#define __cplusplus 201103L
#define GCC_VERSION 40302
#define ARDUINO_ARCH_AVR
#define ARDUINO_AVR_APM_APM_AVR_APM2_2560HAL
#define CONFIG_HAL_BOARD HAL_BOARD_APM2
#define EXCLUDECORE
#define __inline__
#define __asm__(x)
#define __extension__
#define __ATTR_PURE__
#define __ATTR_CONST__
#define __inline__
#define __volatile__
#define __AVR__
typedef void *__builtin_va_list;
#define __builtin_va_start
#define __builtin_va_end
//#define __DOXYGEN__
#define __attribute__(x)
#define NOINLINE __attribute__((noinline))
#define prog_void
#define PGM_VOID_P int

#define NEW_H
            
typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {;}
#undef cli
#define cli()
#include "ArduCopter.pde"
#include "AP_State.pde"
#include "Attitude.pde"
#include "GCS_Mavlink.pde"
#include "Log.pde"
#include "Parameters.pde"
#include "UserCode.pde"
#include "commands.pde"
#include "commands_logic.pde"
#include "compassmot.pde"
#include "compat.pde"
#include "control_acro.pde"
#include "control_althold.pde"
#include "control_auto.pde"
#include "control_autotune.pde"
#include "control_circle.pde"
#include "control_drift.pde"
#include "control_flip.pde"
#include "control_guided.pde"
#include "control_land.pde"
#include "control_loiter.pde"
#include "control_ofloiter.pde"
#include "control_poshold.pde"
#include "control_rtl.pde"
#include "control_sport.pde"
#include "control_stabilize.pde"
#include "crash_check.pde"
#include "ekf_check.pde"
#include "events.pde"
#include "failsafe.pde"
#include "fence.pde"
#include "flight_mode.pde"
#include "heli.pde"
#include "heli_control_acro.pde"
#include "heli_control_stabilize.pde"
#include "inertia.pde"
#include "land_detector.pde"
#include "leds.pde"
#include "motor_test.pde"
#include "motors.pde"
#include "navigation.pde"
#include "perf_info.pde"
#include "position_vector.pde"
#include "radio.pde"
#include "sensors.pde"
#include "setup.pde"
#include "switches.pde"
#include "system.pde"
#include "test.pde"
#endif
