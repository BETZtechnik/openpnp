package org.firepick.driver;

import java.util.List;

import org.openpnp.gui.support.BoundProperty;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.gson.JsonObject;

public class Tmc2130 {
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
	
	private static final Logger logger = LoggerFactory.getLogger(Tmc2130.class);
	FireStepDriver driver;
	public Gconf gConf = new Gconf();
	public Gstat gStat = new Gstat();
	public BoundProperty<Integer> ioin = new BoundProperty<>(0);
	public Ihr ihr = new Ihr();
	public BoundProperty<Integer> tPowerDown = new BoundProperty<>(0);
	public BoundProperty<Integer> tStep = new BoundProperty<>(0);
	public BoundProperty<Integer> tPwmThrs = new BoundProperty<>(0);
	public BoundProperty<Integer> tCoolThrs = new BoundProperty<>(0);
	public BoundProperty<Integer> tHigh = new BoundProperty<>(0);
	public Chopconf chopConf = new Chopconf();
	public Coolconf coolConfig = new Coolconf();
	public Drvstatus drvStatus = new Drvstatus();
	public Pwmconf pwmConf = new Pwmconf();
	public BoundProperty<Integer> pwmScale = new BoundProperty<>(0);
	
	public Tmc2130(FireStepDriver driver) {
		this.driver = driver;
	}
	
	// 0x00 GCONF 
	public class Gconf {
		public BoundProperty<Boolean> en_pwm_mode = new BoundProperty<>(false);
		public BoundProperty<Boolean> small_hysteresis = new BoundProperty<>(false);
		// NOTE: There are other bits in this word, but they can safely be set to zero.
	}
	
	// 0x01 GSTAT
	public class Gstat {
		public BoundProperty<Boolean> reset = new BoundProperty<>(false);
		public BoundProperty<Boolean> drv_err = new BoundProperty<>(false);
		public BoundProperty<Boolean> uv_cp = new BoundProperty<>(false);
	}
	
	// 0x10 IHOLD_RUN
	public class Ihr {
		public BoundProperty<Short> ihold = new BoundProperty<>((short) 0);
		public BoundProperty<Short> irun = new BoundProperty<>((short) 0);
		public BoundProperty<Short> iholddelay = new BoundProperty<>((short) 0);
	}
	
	// 0x6C CHOPCONF
	public class Chopconf {
		public BoundProperty<Boolean> chm = new BoundProperty<>(false);        // Chopper mode.  TRUE = Constant off time with fast decay time.  FALSE=Standard mode (spreadCycle)
		public BoundProperty<Short>   toff = new BoundProperty<>((short) 0);       // TOFF off time & driver enable.  0=Driver disable.  Off time setting controls duration of slow decay phase NCLK= 12 + 32*TOFF
		public BoundProperty<Short>   hstrt = new BoundProperty<>((short) 1);      // 
		public BoundProperty<Short>   hend = new BoundProperty<>((short) 0);       // 
		public BoundProperty<Boolean> fd3 = new BoundProperty<>(false);        // TRUE = 
		public BoundProperty<Boolean> disfdcc = new BoundProperty<>(false);    // Fast decay mode.  TRUE = disfdcc=1 disables current comparator usage for termination of the fast decay cycle
		public BoundProperty<Boolean> rndtf = new BoundProperty<>(false);      // Random TOFF time.  TRUE = Random mode, TOFF is random modulated by dNCLK = -12 ... +3 clocks.
		public BoundProperty<Tbl>     tbl = new BoundProperty<>(Tbl.TBL_16);        // TBL blank time select.
		public BoundProperty<Boolean> vsense = new BoundProperty<>(false);     // TRUE = High sensitivity, low sense resistor voltage.  FALSE = Low sensitivity, high sense resistor voltage.
		public BoundProperty<Boolean> vhighfs = new BoundProperty<>(false);    // TRUE = Enables switching to fullstep, when VHIGH is exceeded. 
		public BoundProperty<Boolean> vhighchm = new BoundProperty<>(false);   // TRUE = TOFF setting automatically becomes doubled during high velocity operation in order to avoid doubling of the chopper frequency.
		public BoundProperty<Short>   sync = new BoundProperty<>((short) 0);       // SYNC PWM synchronization clock. 0=ChopSync off.  0001-1111: Sync with fsync=fck/(sync*64)
		public BoundProperty<Mres>    mres = new BoundProperty<>(Mres.MRES_1);       // Micro-step resolution.  
		public BoundProperty<Boolean> intpol = new BoundProperty<>(false);     // TRUE = The actual microstep resolution (MRES) becomes extrapolated to 256 microsteps for smoothest motor operation
		public BoundProperty<Boolean> dedge = new BoundProperty<>(false);      // TRUE = Enable step impulse at each step edge to reduce step frequency requirement.
		public BoundProperty<Boolean> diss2g = new BoundProperty<>(false);     // TRUE = Short to GND protection is disabled
	}
	
	// 0x6D CO0LCONF
	public class Coolconf {
		public BoundProperty<Boolean> sfilt = new BoundProperty<>(false); // stallGuard2 filter enable
		public BoundProperty<Short>   sgt = new BoundProperty<>((short) 0);     // stallGuard2 threshold value
		public BoundProperty<Boolean> seimin = new BoundProperty<>(false);
		public BoundProperty<Short>   sedn = new BoundProperty<>((short) 0);
		public BoundProperty<Short>   semax = new BoundProperty<>((short) 0);
		public BoundProperty<Short>   seup = new BoundProperty<>((short) 0);
		public BoundProperty<Short>   semin = new BoundProperty<>((short) 0);
	}
	
	// 0x6F DRV_STATUS
	public class Drvstatus {
		public BoundProperty<Boolean> stst = new BoundProperty<>(false);      // standstill indicator
		public BoundProperty<Boolean> olb = new BoundProperty<>(false);       // open load phase A
		public BoundProperty<Boolean> ola = new BoundProperty<>(false);       // open load phase B
		public BoundProperty<Boolean> s2gb = new BoundProperty<>(false);      // short to ground phase B
		public BoundProperty<Boolean> s2ga = new BoundProperty<>(false);      // short to ground phase B
		public BoundProperty<Boolean> otpw = new BoundProperty<>(false);      // overtemperature pre-warning flag
		public BoundProperty<Boolean> ot = new BoundProperty<>(false);        // overtemperature flag
		public BoundProperty<Boolean> stallGuard = new BoundProperty<>(false); // stallGuard 2 status
		public BoundProperty<Short>   cs_actual = new BoundProperty<>((short) 0);  // actual motor current / smart energy current
		public BoundProperty<Boolean> fsactive = new BoundProperty<>(false);  // full step active indicator
		public BoundProperty<Short>   sg_result = new BoundProperty<>((short) 0); // stallGuard 2 result
	}
	
	// 0x70 PWMCONF
	public class Pwmconf {
		public BoundProperty<Short>    pwm_ampl = new BoundProperty<>((short) 0);      // User defined amplitude (offset)
		public BoundProperty<Short>    pwm_grad = new BoundProperty<>((short) 0);      // User defined amplitude (gradient) or regulation loop gradient
		public BoundProperty<Pwmfreq>  pwm_freq = new BoundProperty<>(Pwmfreq.PWMFREQ_1_1024);      // PWM frequency selection
		public BoundProperty<Boolean>   pwm_autoscale = new BoundProperty<>(false); // PWM automatic amplitude scaling
		public BoundProperty<Boolean>   pwm_symmetric = new BoundProperty<>(false); // Force symmetric PWM
		public BoundProperty<Freewheel> freewheel = new BoundProperty<>(Freewheel.FREEWHEEL_FREEWHEEL);     // Allows different standstill modes
	}
	
	// ENCODE / BUILD FUNCTIONS =====================================================
	// Build the 0x00 GCONF word
	public int buildGconf(Gconf gconf) {
		int data = 0;
		data |= (gconf.small_hysteresis.getValue()?1:0     <<14); // 0..3
		data |= (gconf.en_pwm_mode.getValue()?1:0          << 2); // 8..12
		return data;
	}
	// Build the 0x10 IHOLD_RUN word
	public int buildIhr(Ihr ihr){
		int data = 0;
		data |= ((ihr.ihold.getValue()                << 0)  & 0b00000000000000000000000000001111); // 0..3
		data |= ((ihr.irun.getValue()                 << 8)  & 0b00000000000000000001111100000000); // 8..12
		data |= ((ihr.iholddelay.getValue()           << 16) & 0b00000000000011110000000000000000); // 16..19
		return data;
	}
	// Build the 0x6C CHOPCONF word
	public int buildChopconf(Chopconf chopconf) {
		int data = 0;
		data |= ((chopconf.toff.getValue()            << 0)  & 0b00000000000000000000000000001111); // 0..3
		data |= ((chopconf.hstrt.getValue()           << 4)  & 0b00000000000000000000000001110000); // 4..6
		data |= ((chopconf.hend.getValue()            << 7)  & 0b00000000000000000000011110000000); // 7..10
		data |= ((chopconf.fd3.getValue()?1:0         << 11) & 0b00000000000000000000100000000000); // 11
		data |= ((chopconf.disfdcc.getValue()?1:0     << 12) & 0b00000000000000000001000000000000); // 12
		data |= ((chopconf.rndtf.getValue()?1:0       << 13) & 0b00000000000000000010000000000000); // 13
		data |= ((chopconf.chm.getValue()?1:0         << 14) & 0b00000000000000000100000000000000); // 14
		data |= ((chopconf.tbl.getValue().getValue()  << 15) & 0b00000000000000011000000000000000); // 15..16
		data |= ((chopconf.vsense.getValue()?1:0      << 17) & 0b00000000000000100000000000000000); // 17
		data |= ((chopconf.vhighfs.getValue()?1:0     << 18) & 0b00000000000001000000000000000000); // 18
		data |= ((chopconf.vhighchm.getValue()?1:0    << 19) & 0b00000000000010000000000000000000); // 19
		data |= ((chopconf.sync.getValue()            << 20) & 0b00000000111100000000000000000000); // 20..23
		data |= ((chopconf.mres.getValue().getValue() << 24) & 0b00001111000000000000000000000000); // 24..27
		data |= ((chopconf.intpol.getValue()?1:0      << 28) & 0b00010000000000000000000000000000); // 28
		data |= ((chopconf.dedge.getValue()?1:0       << 29) & 0b00100000000000000000000000000000); // 29
		data |= ((chopconf.diss2g.getValue()?1:0      << 30) & 0b01000000000000000000000000000000); // 30
		return data;
	}
	// Build the 0x6D COOLCONF word
	public int buildCoolconf(Coolconf coolconf) {
		int data = 0;
		data |= ((coolconf.semin.getValue()           <<  0) & 0b00000000000000000000000000001111); // 0..3
		data |= ((coolconf.seup.getValue()            <<  5) & 0b00000000000000000000000001100000); // 5..6
		data |= ((coolconf.semax.getValue()           <<  8) & 0b00000000000000000000111100000000); // 8..11
		data |= ((coolconf.sedn.getValue()            << 13) & 0b00000000000000000110000000000000); // 13..14
		data |= ((coolconf.seimin.getValue()?1:0      << 15) & 0b00000000000000001000000000000000); // 15
		data |= ((coolconf.sgt.getValue()             << 16) & 0b00000000011111110000000000000000); // 16..22
		data |= ((coolconf.sfilt.getValue()?1:0       << 24) & 0b00000000000000000000000000000000); // 24
		return data;
	}
	// Build the 0x70 PWMCONF word
	public int buildPwmconf(Pwmconf pwmconf) {
		int data = 0;
		data |= ((pwmconf.pwm_ampl.getValue()             << 0)  & 0b00000000000000000000000011111111); // 0..7
		data |= ((pwmconf.pwm_grad.getValue()             << 8)  & 0b00000000000000001111111100000000); // 8..15
		data |= ((pwmconf.pwm_freq.getValue().getValue()  << 16) & 0b00000000000000110000000000000000); // 16..17
		data |= ((pwmconf.pwm_autoscale.getValue()?1:0    << 18) & 0b00000000000001000000000000000000); // 18
		data |= ((pwmconf.pwm_symmetric.getValue()?1:0    << 19) & 0b00000000000010000000000000000000); // 19
		data |= ((pwmconf.freewheel.getValue().getValue() << 20) & 0b00000000001100000000000000000000); // 20..21
		return data;
	}
	
	// DECODE FUNCTIONS=======================================================
	// Decode the 0x01 GSTAT word
	public Gstat decodeGstat(int rawval) {
		Gstat gstat = new Gstat();
		gstat.drv_err.setValue((((rawval >> 0) & 0x01) != 0));
		gstat.reset.setValue((((rawval >> 1) & 0x01) != 0));
		gstat.uv_cp.setValue((((rawval >> 2) & 0x01) != 0)); 
		return gstat;
	}
	// Decode the 0x6F DRV_STATUS word
	public Drvstatus decodeDrvstatus(int data) {
		Drvstatus drvstatus = new Drvstatus();
		drvstatus.stst.setValue( ((data      ) & 0b00000000000000000000000111111111) != 0);
		drvstatus.olb.setValue( ((data >> 15) & 0b00000000000000000000000000000001) != 0);
		drvstatus.ola.setValue( ((data >> 16) & 0b00000000000000000000000000011111) != 0);
		drvstatus.s2gb.setValue( ((data >> 24) & 0b00000000000000000000000000000001) != 0);
		drvstatus.s2ga.setValue( ((data >> 25) & 0b00000000000000000000000000000001) != 0);
		drvstatus.otpw.setValue( ((data >> 26) & 0b00000000000000000000000000000001) != 0);
		drvstatus.ot.setValue( ((data >> 27) & 0b00000000000000000000000000000001) != 0);
		drvstatus.stallGuard.setValue( ((data >> 28) & 0b00000000000000000000000000000001) != 0);
		drvstatus.cs_actual.setValue((short)  ((data >> 29) & 0b00000000000000000000000000000001) );
		drvstatus.fsactive.setValue(( ((data >> 30) & 0b00000000000000000000000000000001) != 0));
		drvstatus.sg_result.setValue((short)  ((data >> 31) & 0b00000000000000000000000000000001));
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
	public void setGconf(int gConf) throws Exception {
	    logger.trace(String.format("FireStep: TMC2130 set GCONF to %d", gConf ));
        try {
        	driver.sendJsonCommand(String.format("{'tmcgconf':%s}", gConf));
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
	public void setCoolConf(int coolconf) throws Exception {
	    logger.trace(String.format("FireStep: TMC2130 set COOLCONF to %d", coolconf ));
        try {
        	driver.sendJsonCommand(String.format("{'tmcccoolc':%s}", coolconf));
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	public void setIhr(int ihr) throws Exception {
	    logger.trace(String.format("FireStep: TMC2130 set IHR to %d", ihr ));
        try {
        	driver.sendJsonCommand(String.format("{'tmcihr':%s}", ihr));
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
	
	
	
	
	// These are the methods that are called as actions from the wizard.
	public void writeGconf() throws Exception {
    	driver.sendJsonCommand(String.format("{'tmcgconf':%d}", buildGconf(this.gConf)));
    	// Error: -402
	}
	
	public void readGstat() throws Exception {
		JsonObject response = driver.sendJsonCommand(String.format("{'tmcgstat':''}")).get(0);
		// Error: -402
	}
	
	public void readIoin() throws Exception {
		JsonObject response = driver.sendJsonCommand(String.format("{'tmcvers':''}")).get(0);
		// Error: -402
	}
	
	public void writeIholdDelay() throws Exception {
    	driver.sendJsonCommand(String.format("{'tmcihr':%d}", buildIhr(this.ihr)));
    	// Error: -402
	}
	
	public void writeTpowerDown() throws Exception {
    	driver.sendJsonCommand(String.format("{'tmctpwrdn':%d}", this.tPowerDown.getValue()));
    	// Responds with plain text, breaking JSON protocol.
    	// '{"s":0,"r":{"tmctpwrdn":0},"t":0.001} '
	}
	
	public void readTstep() throws Exception {
		JsonObject response = driver
				.sendJsonCommand(String.format("{'tmctstep':''}"))
				.get(0)
				.get("r")
				.getAsJsonObject()
				.get("tmctstep")
				.getAsJsonObject();
		// {"tmctstep":""} => {"s":0,"r":{"tmctstep":{"x":65535,"y":65535,"z":65535}},"t":0.031}
		this.tStep.setValue(response.get("x").getAsInt());
	}
	
	public void writeTpwmThrs() throws Exception {
    	driver.sendJsonCommand(String.format("{'tmctpwmth':%d}", this.tPwmThrs.getValue()));    	
    	// Responds with plain text, breaking JSON protocol.
	}
	
	public void writeTcoolThrs() throws Exception {
    	driver.sendJsonCommand(String.format("{'tmctcthrs':%d}", this.tCoolThrs.getValue()));
    	// Error: -402
	}
	
	public void writeThigh() throws Exception {
    	driver.sendJsonCommand(String.format("{'tmcthigh':%d}", this.tHigh.getValue()));
    	// Responds with plain text, breaking JSON protocol.
    	// {"tmctcthrs":0} => [{"s":-402,"r":{"tmctcthrs":0},"e":"tmctcth","t":0.001}]
	}
	
	public void writeChopConf() throws Exception {
    	driver.sendJsonCommand(String.format("{'tmccconf':%d}", buildChopconf(this.chopConf)));
    	// Responds with plain text, breaking JSON protocol.
	}
	
	public void writeCoolConf() throws Exception {
    	driver.sendJsonCommand(String.format("{'tmccoolc':%d}", buildCoolconf(this.coolConfig)));
    	// Error: -402
	}
	
	public void readDrvStatus() throws Exception {
		JsonObject response = driver.sendJsonCommand(String.format("{'tmcdrvst':''}")).get(0);
		// Error: -402
	}
	
	public void writePwmConf() throws Exception {
    	driver.sendJsonCommand(String.format("{'tmcpconf':%d}", buildPwmconf(this.pwmConf)));
    	// Responds with plain text, breaking JSON protocol.
	}
	
	public void readPwmScale() throws Exception {
		JsonObject response = driver
				.sendJsonCommand(String.format("{'tmcpscale':''}"))
				.get(0)
				.get("r")
				.getAsJsonObject()
				.get("tmcpscale")
				.getAsJsonObject();
		// {'tmcpscale':''} => {"s":0,"r":{"tmcpscale":{"x":255,"y":255,"z":255}},"t":0.031}
		this.pwmScale.setValue(response.get("x").getAsInt());
	}
} 
// End TMC2130