package org.firepick.driver;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Tmc2130 {
	private static final Logger logger = LoggerFactory.getLogger(Tmc2130.class);
	FireStepDriver driver;
	public Tmc2130(FireStepDriver driver) {
		this.driver = driver;
	}
	
	// 0x00 GCONF 
	public class Gconf {
		boolean en_pwm_mode;
		boolean small_hysteresis;
		// NOTE: There are other bits in this word, but they can safely be set to zero.
	}
	
	// 0x01 GSTAT
	public class Gstat {
		boolean reset;
		boolean drv_err;
		boolean uv_cp;
	}
	
	// 0x10 IHOLD_RUN
	public class Ihr {
		short ihold;
		short irun;
		short iholddelay;
	}
	
	// CHOPCONF -> MRES - Micro-step resolution
	public enum Mres {
		MRES_256(0), // 0 : Native 256 microstep setting
		MRES_128(1), // 1 : 128x microstepping
		MRES_64 (2), // 2 : 64x microstepping
		MRES_32 (3), // 3 : 32x microstepping
		MRES_16 (4), // 4 : 16x microstepping
		MRES_8  (5), // 5 : 8x microstepping
		MRES_4  (6), // 6 : 4x microstepping
		MRES_2  (7), // 7 : 2x microstepping
		MRES_1  (8); // 8 : 1x full-step
		private final int value;
		private Mres(int mres) {
			this.value = mres;
		}
		public int getValue() {
			return value;
		}
	};

	// CHOPCONF -> TBL - TBL blank time select.  16, 24, 36, or 54 clock cycles.
	public enum Tbl {
		TBL_16(1),
		TBL_24(2),      // Recommended for most applications.
		TBL_36(3),      // Recommended for most applications.
		TBL_54(4);
		private final int value;
		private Tbl(int tbl) {
			this.value = tbl;
		}
		public int getValue() {
			return value;
		}
	};
	// 0x6C CHOPCONF
	public class Chopconf {
		boolean chm;        // Chopper mode.  TRUE = Constant off time with fast decay time.  FALSE=Standard mode (spreadCycle)
		short   toff;       // TOFF off time & driver enable.  0=Driver disable.  Off time setting controls duration of slow decay phase NCLK= 12 + 32*TOFF
		short   hstrt;      // 
		short   hend;       // 
		boolean fd3;        // TRUE = 
		boolean disfdcc;    // Fast decay mode.  TRUE = disfdcc=1 disables current comparator usage for termination of the fast decay cycle
		boolean rndtf;      // Random TOFF time.  TRUE = Random mode, TOFF is random modulated by dNCLK = -12 ... +3 clocks.
		Tbl     tbl;        // TBL blank time select.
		boolean vsense;     // TRUE = High sensitivity, low sense resistor voltage.  FALSE = Low sensitivity, high sense resistor voltage.
		boolean vhighfs;    // TRUE = Enables switching to fullstep, when VHIGH is exceeded. 
		boolean vhighchm;   // TRUE = TOFF setting automatically becomes doubled during high velocity operation in order to avoid doubling of the chopper frequency.
		short   sync;       // SYNC PWM synchronization clock. 0=ChopSync off.  0001-1111: Sync with fsync=fck/(sync*64)
		Mres    mres;       // Micro-step resolution.  
		boolean intpol;     // TRUE = The actual microstep resolution (MRES) becomes extrapolated to 256 microsteps for smoothest motor operation
		boolean dedge;      // TRUE = Enable step impulse at each step edge to reduce step frequency requirement.
		boolean diss2g;     // TRUE = Short to GND protection is disabled
	}
	
	// 0x6D CO0LCONF
	public class Coolconf {
		boolean sfilt; // stallGuard2 filter enable
		short   sgt;     // stallGuard2 threshold value
		boolean seimin;
		short   sedn;
		short   semax;
		short   seup;
		short   semin;
	}
	
	// 0x6F DRV_STATUS
	public class Drvstatus {
		boolean stst;      // standstill indicator
		boolean olb;       // open load phase A
		boolean ola;       // open load phase B
		boolean s2gb;      // short to ground phase B
		boolean s2ga;      // short to ground phase B
		boolean otpw;      // overtemperature pre-warning flag
		boolean ot;        // overtemperature flag
		boolean stallGuard; // stallGuard 2 status
		short   cs_actual;  // actual motor current / smart energy current
		boolean fsactive;  // full step active indicator
		short   sg_result; // stallGuard 2 result
	}
	
	// PWMCONF -> Freewheel
	public enum Freewheel {
		FREEWHEEL_NORMAL (1),  // %00: Normal operation
		FREEWHEEL_FREEWHEEL(2),// %01: Freewheeling
		FREEWHEEL_SHORT_LS(3), // %10: Coil shorted using LS drivers
		FREEWHEEL_SHORT_HS(4); // %11: Coil shorted using HS drivers
		private final int value;
		private Freewheel(int freewheel) {
			this.value = freewheel;
		}
		public int getValue() {
			return value;
		}
	};
	
	// PWMCONF -> PwmFreq
	public enum Pwmfreq {
		PWMFREQ_1_1024 (0), // %00: f PWM =1/1024 f CLK
		PWMFREQ_1_683  (1), // %01: f PWM =1/683 f CLK
		PWMFREQ_1_512  (2), // %10: f PWM =1/512 f CLK
		PWMFREQ_1_410  (3);  // %11: f PWM =1/410 f CLK
		private final int value;
		private Pwmfreq(int pwmfreq) {
			this.value = pwmfreq;
		}
		public int getValue() {
			return value;
		}
	};
	
	// 0x70 PWMCONF
	public class Pwmconf {
		short    pwm_ampl;      // User defined amplitude (offset)
		short    pwm_grad;      // User defined amplitude (gradient) or regulation loop gradient
		Pwmfreq  pwm_freq;      // PWM frequency selection
		boolean  pwm_autoscale; // PWM automatic amplitude scaling
		boolean  pwm_symmetric; // Force symmetric PWM
		Freewheel freewheel;     // Allows different standstill modes
	}
	
	// ENCODE / BUILD FUNCTIONS =====================================================
	// Build the 0x00 GCONF word
	public int buildGconf(Gconf gconf) {
		int data = 0;
		data |= (gconf.small_hysteresis?1:0     <<14); // 0..3
		data |= (gconf.en_pwm_mode?1:0          << 2); // 8..12
		return data;
	}
	// Build the 0x10 IHOLD_RUN word
	public int buildIhr(Ihr ihr){
		int data = 0;
		data |= ((ihr.ihold                << 0)  & 0b00000000000000000000000000001111); // 0..3
		data |= ((ihr.irun                 << 8)  & 0b00000000000000000001111100000000); // 8..12
		data |= ((ihr.iholddelay           << 16) & 0b00000000000011110000000000000000); // 16..19
		return data;
	}
	// Build the 0x6C CHOPCONF word
	public int buildChopconf(Chopconf chopconf) {
		int data = 0;
		data |= ((chopconf.toff            << 0)  & 0b00000000000000000000000000001111); // 0..3
		data |= ((chopconf.hstrt           << 4)  & 0b00000000000000000000000001110000); // 4..6
		data |= ((chopconf.hend            << 7)  & 0b00000000000000000000011110000000); // 7..10
		data |= ((chopconf.fd3?1:0         << 11) & 0b00000000000000000000100000000000); // 11
		data |= ((chopconf.disfdcc?1:0     << 12) & 0b00000000000000000001000000000000); // 12
		data |= ((chopconf.rndtf?1:0       << 13) & 0b00000000000000000010000000000000); // 13
		data |= ((chopconf.chm?1:0         << 14) & 0b00000000000000000100000000000000); // 14
		data |= ((chopconf.tbl.getValue()  << 15) & 0b00000000000000011000000000000000); // 15..16
		data |= ((chopconf.vsense?1:0      << 17) & 0b00000000000000100000000000000000); // 17
		data |= ((chopconf.vhighfs?1:0     << 18) & 0b00000000000001000000000000000000); // 18
		data |= ((chopconf.vhighchm?1:0    << 19) & 0b00000000000010000000000000000000); // 19
		data |= ((chopconf.sync            << 20) & 0b00000000111100000000000000000000); // 20..23
		data |= ((chopconf.mres.getValue() << 24) & 0b00001111000000000000000000000000); // 24..27
		data |= ((chopconf.intpol?1:0      << 28) & 0b00010000000000000000000000000000); // 28
		data |= ((chopconf.dedge?1:0       << 29) & 0b00100000000000000000000000000000); // 29
		data |= ((chopconf.diss2g?1:0      << 30) & 0b01000000000000000000000000000000); // 30
		return data;
	}
	// Build the 0x6D COOLCONF word
	public int buildCoolconf(Coolconf coolconf) {
		int data = 0;
		data |= ((coolconf.semin           <<  0) & 0b00000000000000000000000000001111); // 0..3
		data |= ((coolconf.seup            <<  5) & 0b00000000000000000000000001100000); // 5..6
		data |= ((coolconf.semax           <<  8) & 0b00000000000000000000111100000000); // 8..11
		data |= ((coolconf.sedn            << 13) & 0b00000000000000000110000000000000); // 13..14
		data |= ((coolconf.seimin?1:0      << 15) & 0b00000000000000001000000000000000); // 15
		data |= ((coolconf.sgt             << 16) & 0b00000000011111110000000000000000); // 16..22
		data |= ((coolconf.sfilt?1:0       << 24) & 0b00000000000000000000000000000000); // 24
		return data;
	}
	// Build the 0x70 PWMCONF word
	public int buildPwmconf(Pwmconf pwmconf) {
		int data = 0;
		data |= ((pwmconf.pwm_ampl             << 0)  & 0b00000000000000000000000011111111); // 0..7
		data |= ((pwmconf.pwm_grad             << 8)  & 0b00000000000000001111111100000000); // 8..15
		data |= ((pwmconf.pwm_freq.getValue()  << 16) & 0b00000000000000110000000000000000); // 16..17
		data |= ((pwmconf.pwm_autoscale?1:0    << 18) & 0b00000000000001000000000000000000); // 18
		data |= ((pwmconf.pwm_symmetric?1:0    << 19) & 0b00000000000010000000000000000000); // 19
		data |= ((pwmconf.freewheel.getValue() << 20) & 0b00000000001100000000000000000000); // 20..21
		return data;
	}
	
	// DECODE FUNCTIONS=======================================================
	// Decode the 0x01 GSTAT word
	public Gstat decodeGstat(int rawval) {
		Gstat gstat = new Gstat();
		gstat.drv_err = (((rawval >> 0) & 0x01) != 0);
		gstat.reset   = (((rawval >> 1) & 0x01) != 0);
		gstat.uv_cp   = (((rawval >> 2) & 0x01) != 0); 
		return gstat;
	}
	// Decode the 0x6F DRV_STATUS word
	public Drvstatus decodeDrvstatus(int data) {
		Drvstatus drvstatus = new Drvstatus();
		drvstatus.stst       =        ( ((data      ) & 0b00000000000000000000000111111111) != 0);
		drvstatus.olb        =        ( ((data >> 15) & 0b00000000000000000000000000000001) != 0);
		drvstatus.ola        =        ( ((data >> 16) & 0b00000000000000000000000000011111) != 0);
		drvstatus.s2gb       =        ( ((data >> 24) & 0b00000000000000000000000000000001) != 0);
		drvstatus.s2ga       =        ( ((data >> 25) & 0b00000000000000000000000000000001) != 0);
		drvstatus.otpw       =        ( ((data >> 26) & 0b00000000000000000000000000000001) != 0);
		drvstatus.ot         =        ( ((data >> 27) & 0b00000000000000000000000000000001) != 0);
		drvstatus.stallGuard =        ( ((data >> 28) & 0b00000000000000000000000000000001) != 0);
		drvstatus.cs_actual  = (short)  ((data >> 29) & 0b00000000000000000000000000000001) ;
		drvstatus.fsactive   =        ( ((data >> 30) & 0b00000000000000000000000000000001) != 0);
		drvstatus.sg_result  = (short)  ((data >> 31) & 0b00000000000000000000000000000001) ;
		return drvstatus;
	}

	// WRITE FUNCTIONS ========================================================
	public void setIhold(int ihold) throws Exception {
	    logger.trace(String.format("FireStep: TMC2130 set IHOLD to %d", ihold ));
        try {
        	driver.sendJsonCommand(String.format("{'tmcihold':%s}", ihold));
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	public void setIrun(int irun) throws Exception {
	    logger.trace(String.format("FireStep: TMC2130 set IRUN to %d", irun ));
        try {
        	driver.sendJsonCommand(String.format("{'tmcirun':%s}", irun));
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	public void setIholdDelay(int iholddelay) throws Exception {
	    logger.trace(String.format("FireStep: TMC2130 set IHOLDDELAY to %d", iholddelay ));
        try {
        	driver.sendJsonCommand(String.format("{'tmcihdly':%s}", iholddelay));
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	public void setTpowerDown(int tpowerdown) throws Exception {
	    logger.trace(String.format("FireStep: TMC2130 set TPOWERDOWN to %d", tpowerdown ));
        try {
        	driver.sendJsonCommand(String.format("{'tmctpwrdn':%s}", tpowerdown));
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	public void setTpwmThrs(int tpwmthrs) throws Exception {
	    logger.trace(String.format("FireStep: TMC2130 set TPWMTHRS to %d", tpwmthrs ));
        try {
        	driver.sendJsonCommand(String.format("{'tmctpwmth':%s}", tpwmthrs));
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	public void setThigh(int thigh) throws Exception {
	    logger.trace(String.format("FireStep: TMC2130 set THIGH to %d", thigh ));
        try {
        	driver.sendJsonCommand(String.format("{'tmcthigh':%s}", thigh));
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void setChopConf(int chopconf) throws Exception {
	    logger.trace(String.format("FireStep: TMC2130 set CHOPCONF to %d", chopconf ));
        try {
        	driver.sendJsonCommand(String.format("{'tmccconf':%s}", chopconf));
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	public void setPwmConf(int pwmconf) throws Exception {
	    logger.trace(String.format("FireStep: TMC2130 set PWMCONF to %d", pwmconf ));
        try {
        	driver.sendJsonCommand(String.format("{'tmcpconf':%s}", pwmconf));
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	//READ FUNCTIONS=======================================================
	public void getLoadMeas(int pin, boolean state) throws Exception {
		int loadmeasX = 0;
		int loadmeasY = 0;
		int loadmeasZ = 0;
        try {
        	driver.sendJsonCommand("{'tmcload':''}");
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	    logger.trace(String.format("FireStep: TMC2130 get LoadMeas: x=%d, y=%d, z=%d", loadmeasX, loadmeasY, loadmeasZ ));
	}
	
	public void getTstep(int pin, boolean state) throws Exception {
		int tstepX = 0;
		int tstepY = 0;
		int tstepZ = 0;
        try {
        	driver.sendJsonCommand("{'tmctstep':''}");
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	    logger.trace(String.format("FireStep: TMC2130 get TSTEP: x=%d, y=%d, z=%d", tstepX, tstepY, tstepZ ));
	}
	public void getPwmScale(int pin, boolean state) throws Exception {
		int pwmScaleX = 0;
		int pwmScaleY = 0;
		int pwmScaleZ = 0;
        try {
        	driver.sendJsonCommand("{'tmcpscale':''}");
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	    logger.trace(String.format("FireStep: TMC2130 get PWM_SCALE: x=%d, y=%d, z=%d", pwmScaleX, pwmScaleY, pwmScaleZ ));
	}
} 
// End TMC2130