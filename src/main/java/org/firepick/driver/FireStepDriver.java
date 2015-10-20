/*
	Copyright (C) 2011 Jason von Nieda <jason@vonnieda.org>
	
	This file is part of OpenPnP.
	
OpenPnP is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

OpenPnP is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with OpenPnP.  If not, see <http://www.gnu.org/licenses/>.
	
	For more information about OpenPnP visit http://openpnp.org
*/

// This file is intended to support the FireStep motion controller, created by Karl Lew (karl@firepick.org).
// More information about the FireStep controller can be found at https://github.com/firepick1/firestep
// Note that this implementation currently only supports FirePick Delta, which has rotational delta kinematics.
// It should be trivial to add conditional hooks to enable or disable or switch kinematics for other configurations.
// - Neil Jansen (njansen1@gmail.com) 7/1/2014

package org.firepick.driver;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.Callable;
import java.util.concurrent.Future;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.swing.Action;

import org.firepick.driver.FireStepDriver.BarycentricCalibration.DomainAndRange;
import org.firepick.driver.wizards.FireStepDriverWizard;
import org.firepick.feeder.CarouselFeeder;
import org.firepick.kinematics.RotatableDeltaKinematicsCalculator;
import org.firepick.model.RawStepTriplet;
import org.firepick.vision.FireSight;
import org.firepick.vision.FireSight.FireSightResult;
import org.openpnp.ConfigurationListener;
import org.openpnp.gui.MainFrame;
import org.openpnp.gui.support.PropertySheetWizardAdapter;
import org.openpnp.gui.support.Wizard;
import org.openpnp.machine.reference.ReferenceActuator;
import org.openpnp.machine.reference.ReferenceCamera;
import org.openpnp.machine.reference.ReferenceHead;
import org.openpnp.machine.reference.ReferenceHeadMountable;
import org.openpnp.machine.reference.ReferenceMachine;
import org.openpnp.machine.reference.ReferenceNozzle;
import org.openpnp.machine.reference.ReferencePasteDispenser;
import org.openpnp.machine.reference.driver.AbstractSerialPortDriver;
import org.openpnp.model.Configuration;
import org.openpnp.model.Length;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.openpnp.model.Point;
import org.openpnp.spi.Camera;
import org.openpnp.spi.Head;
import org.openpnp.spi.HeadMountable;
import org.openpnp.spi.Machine;
import org.openpnp.spi.Nozzle;
import org.openpnp.spi.PasteDispenser;
import org.openpnp.spi.PropertySheetHolder;
import org.openpnp.util.MovableUtils;
import org.openpnp.util.VisionUtils;
import org.simpleframework.xml.Attribute;
import org.simpleframework.xml.Element;
import org.simpleframework.xml.ElementList;
import org.simpleframework.xml.ElementMap;
import org.simpleframework.xml.core.Commit;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.util.concurrent.FutureCallback;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;


public class FireStepDriver extends AbstractSerialPortDriver implements Runnable {
	private static final Logger logger = LoggerFactory.getLogger(FireStepDriver.class);
	private static final double minimumRequiredVersion = 1.0;
	
	@Element(required=false)
	private boolean usePwmVacuum = false; // Intended to provide legacy support for older FireStep versions.. might remove this later.

	@Element(required=false)
	private int pwmVacuumSetting = 50;   // Provides a default value.  This might change later depending on whether we want to 
	                                     // run the pump harder for larger parts (might depend on nozzle, part size, etc..).
	
	@Element(required=false)
	private RotatableDeltaKinematicsCalculator deltaCalculator = new RotatableDeltaKinematicsCalculator();
	
	@Attribute(required=false)
	private boolean autoUpdateToolOffsets = false;

    @Element(required=false)
	private BarycentricCalibration barycentricCalibration;
    
	@ElementList(required=false)
	private List<CarouselDriver> carouselDrivers = new ArrayList<CarouselDriver>();
	
	@Attribute(required=false)
	private long dispenseTimeMilliseconds = 400;
	
	@Attribute(required=false)
	private boolean useFireStepKinematics = true;
	
	@Element(required=false)
	private CameraPoseCalibration cameraPoseCalibration = new CameraPoseCalibration();
	
    @Attribute(required=false)
	private double nozzleStepsPerDegree =  8.888888888;
    
    @Element(required=false)
    private Tmc2130 tmc2130 = new Tmc2130();
    
    
    private boolean nozzleEnabled = false;
    private boolean powerSupplyOn = false;
    private BarycentricInterpolation barycentric;
    private ReferenceHeadMountable lastToolUsed;
    private int rawFeedrate = 12800; //12800 is FireStep's default feedrate
    private double x, y, z, c;
    private Thread readerThread;
    private boolean disconnectRequested;
    private boolean connected;
    private String connectedVersion;
    private LinkedBlockingQueue<JsonObject> responseQueue = new LinkedBlockingQueue<>();
    private JsonParser parser = new JsonParser();
	/*
	 * Stores whether or not the machine has been homed. If it has been homed
	 * and we are disabling the machine, before homing again we move to zero
	 * at Safe-Z so we don't hit the machine extents.
	 */
	private boolean homed = false;
	
	public FireStepDriver() {
	    Configuration.get().addListener(new ConfigurationListener() {
            @Override
            public void configurationLoaded(Configuration configuration)
                    throws Exception {
                // TODO Auto-generated method stub
                
            }
            
            @Override
            public void configurationComplete(Configuration configuration)
                    throws Exception {
                ReferenceMachine machine = (ReferenceMachine) configuration.getMachine();
                machine.registerFeederClass(CarouselFeeder.class);
                
                tmc2130.setDriver(FireStepDriver.this);
                
                if (carouselDrivers.isEmpty()) {
                	carouselDrivers.add(new CarouselDriver());
                }
                
                for (CarouselDriver driver : carouselDrivers) {
                    driver.setMachineDriver(FireStepDriver.this);
                }
            }
        });
	}
	
	public Tmc2130 getTmc2130() {
		return tmc2130;
	}
	
	@Commit
	private void commit() {
	    if (barycentricCalibration == null) {
	        barycentricCalibration = new BarycentricCalibration(true);
	    }
	}
	
	public CarouselDriver getCarouselDriver(int address) {
	    for (CarouselDriver driver : carouselDrivers) {
	        if (driver.getAddress() == address) {
	            return driver;
	        }
	    }
	    return null;
	}
	
	@Override
	public void setEnabled(boolean enabled) throws Exception {
	    if (enabled) {
            if (barycentricCalibration.enabled) {
                barycentric = new BarycentricInterpolation(barycentricCalibration.getMap());
            }
            else {
                barycentric = null;
            }
	        if (!connected) {
	            try {
	                connect();
	            }
	            catch (Exception e) {
	                e.printStackTrace();
	                throw e;
	            }
	        }
	        enableVacuumPump(false);              		// Turn the vacuum pump OFF
	        enableDispenser(false);
	        enablePowerSupply(true);              		// Turn power supply ON
	        if (powerSupplyOn)					  		// Exception should catch but guard just in case
	        {
				Thread.sleep(500,0);                	// Delay for a bit, wait for power supply to stabilize.
				setXyzMotorEnable(true);				// Enable power for XYZ stepper motors
	    		enableEndEffectorRingLight(true); 		// Turn off down-looking LED ring light
				Thread.sleep(50,0);                 	// Delay for a bit, wait for power supply to stabilize.
		        setFireStepKinematicsEnabled(useFireStepKinematics);
		        home(null);                         	// home the machine
		        for (CarouselDriver carouselDriver : carouselDrivers) {
		            carouselDriver.home();
		        }
	        }
			
	    } //if (enabled)
	    else{
	    	if (connected)
	    	{
	    		enableEndEffectorRingLight(false); 		// Turn off down-looking LED ring light
	    		enableUpLookingRingLight(false);   		// Turn off up-looking LED ring light
	    		if (powerSupplyOn)
	    		{
			        home(null);                        	// home the machine
                    enableVacuumPump(false);            // Turn the vacuum pump OFF
                    enableDispenser(false);
					setXyzMotorEnable(false);  			// Disable power for XYZ stepper motors
			        enablePowerSupply(false);          	// Turn off the power supply
	    		}
	    	}
	    	homed = false;
	    }
	}
	
	@Override
	public void actuate(ReferenceActuator actuator, boolean on)
			throws Exception {
		if (actuator.getIndex() == 0) {
			//TODO: Currently disabled... We don't have a pin to assign this to
		}
	}
	
	@Override
    public void dispense(ReferencePasteDispenser dispenser,
            Location startLocation, Location endLocation,
            long dispenseTimeMilliseconds) throws Exception {
	    enableDispenser(true);
	    Thread.sleep(this.dispenseTimeMilliseconds);
        enableDispenser(false);
    }

    @Override
	public void home(ReferenceHead head) throws Exception {
    	Camera camera = Configuration
    			.get()
    			.getMachine()
    			.getDefaultHead()
    			.getDefaultCamera();
    	
    	// If we've previously homed we want to make sure the head is not in
    	// a position that will cause arm collision if we home again. So we
    	// try to move back to center at a safe height before homing.
        if (homed) {
            if (camera.getLocation().getLinearDistanceTo(0, 0) > 60) {
            	camera.moveToSafeZ(1.0);
            	camera.moveTo(camera.getLocation().derive(0d, 0d, null, null), 1.0);
            }
        }
        
        RawStepTriplet rs = deltaCalculator.getHomeRawSteps();
        if (useFireStepKinematics) {
            // find the smallest home angle step count
            int hascMin = Math.min(rs.x, Math.min(rs.y, rs.z));
            
            sendJsonCommand(String.format("{'hom':{'x':%d,'y':%d,'z':%d}}",
                    -hascMin,
                    -hascMin,
                    -hascMin), 10000);
        }
        else {
            sendJsonCommand(String.format("{'hom':{'x':%d,'y':%d,'z':%d}}", -rs.x, -rs.y, -rs.z), 10000);
        }
        setLocation(getFireStepLocation());
        homed = true;
	}
	
	@Override
	public Location getLocation(ReferenceHeadMountable hm) {
		return new Location(LengthUnit.Millimeters, x, y, z, c).add(hm.getHeadOffsets());
	}
	
	@Override
	public void actuate(ReferenceActuator actuator, double value)
	  throws Exception {
	  	//dwell();
	    // TODO Auto-generated method stub
	}
	
	@Override
	public void moveTo(ReferenceHeadMountable hm, Location location, double speed)
			throws Exception {
	    // If we're moving the nozzle or paste dispenser for the first time
	    // or for the first time since moving the other, we check it's
	    // offsets, since the process of inserting the tool will have
	    // changed them.
	    if (autoUpdateToolOffsets 
	            && lastToolUsed != hm 
	            && (hm instanceof Nozzle || hm instanceof PasteDispenser)) {
	        boolean autoUpdateToolOffsets = this.autoUpdateToolOffsets;
	        this.autoUpdateToolOffsets = false;
	        try {
	            updateToolOffsets(hm);
	        }
	        finally {
	            this.autoUpdateToolOffsets = autoUpdateToolOffsets;
	        }
	        lastToolUsed = hm;
	    }
	    
	    location = location.subtract(hm.getHeadOffsets());
	    location = location.convertToUnits(LengthUnit.Millimeters);
	    // Update the target location to handle any NaNs that were supplied
	    location = location.derive(
                Double.isNaN(location.getX()) ? this.x : location.getX(),
                Double.isNaN(location.getY()) ? this.y : location.getY(),
                Double.isNaN(location.getZ()) ? this.z : location.getZ(),
                Double.isNaN(location.getRotation()) ? this.c : location.getRotation());
        Location scaledLocation = location.derive(null, null, null, null);
        if (barycentricCalibration.enabled) {
            scaledLocation = barycentric.interpolate(scaledLocation);
        }
        
        if (cameraPoseCalibration.enabled) {
            // Calculate the offset per unit
            Location offset = cameraPoseCalibration.bottom.subtract(cameraPoseCalibration.top);
            offset = offset.multiply(1 / offset.getZ(), 1 / offset.getZ(), 0, 0);
            // Determine how far in Z we are from the start point
            Location delta = scaledLocation.subtract(cameraPoseCalibration.top);
            // And scale the offset by the distance in Z
            offset = offset.multiply(delta.getZ(), delta.getZ(), 0, 0);
            scaledLocation = scaledLocation.add(offset);
        }
	    
        if (useFireStepKinematics) {
            moveToFireStepKinematics(hm, scaledLocation, speed);
        }
        else {
            moveToRaw(hm, scaledLocation, speed);
        }

        // TODO: Since we handle the NaNs up top we can probably skip all
        // these checks, but think about it a bit first.
	    if (!Double.isNaN(location.getX())) {
	        this.x = location.getX();
	    }
	    if (!Double.isNaN(location.getY())) {
	        this.y = location.getY();
	    }
	    if (!Double.isNaN(location.getZ())) {
	        this.z = location.getZ();
	    }
	    if (!Double.isNaN(location.getRotation())) {
	        this.c = location.getRotation();
	    }
	}
	
    public void moveToRaw(ReferenceHeadMountable hm, Location location, double speed)
            throws Exception {
        logger.debug("moveToRaw {}", location);
        int rotSteps = 0;
        RawStepTriplet rs = new RawStepTriplet(0,0,0);
        boolean moveXyz = false;
        boolean moveRot = false;
        
        //Check if we've rotated
        if (Math.abs(location.getRotation() - this.c) >= 0.01)
        {
            moveRot = true;
            //Convert the rotation axis from degrees to steps
            rotSteps = (int)(location.getRotation() * nozzleStepsPerDegree + 0.5d);
            if ((rotSteps >= 32000) || (rotSteps <= -32000)) {
                throw new Error(String.format("FireStep: Rotation axis raw position cannot exceed +/- 32000 steps",rotSteps));
            }
        }
        
        //Check if we've moved in XYZ
        /**
         * We must compare the target location to the current location. Target
         * location has to be created and take the NaN's into account. Use derive?
         */
        Location currentLoc = new Location(LengthUnit.Millimeters, this.x, this.y, this.z, 0);
        if (Math.abs(location.getXyzDistanceTo(currentLoc)) >= 0.01) {
            moveXyz = true;
            logger.trace(String.format("moveTo Cartesian: X: %.3f, Y: %.3f, Z: %.3f",location.getX(), location.getY(),location.getZ() ));
            
            // Convert angles into raw steps
            rs = deltaCalculator.getRawSteps(location);
            logger.trace(String.format("moveTo RawSteps: X: %d, Y: %d, Z: %d",rs.x, rs.y,rs.z ));
        }
        
        
        // Get feedrate in raw steps
        // Note that speed is defined by (maximum feed rate * speed) where speed is greater than 0 and typically less than or equal to 1. 
        // A speed of 0 means to move at the minimum possible speed.
        //TODO: Set feedrate based in raw steps, based off of 'feedRateMmPerMinute' and 'speed'
        // 'mv' is maximum velocity (pulses/second), and the default is 12800.

        rawFeedrate = (int)((double)rawFeedrate * speed); //Multiply rawFeedrate by speed, which should be 0 to 1
        if (moveXyz){
            if (moveRot){ // Cartesian move with rotation.  Feedrate is (TBD)
                logger.trace(String.format("moveTo: Cartesian move with rotation, feedrate=%d steps/second",rawFeedrate));
                setRotMotorEnable(true);
                sendJsonCommand(String.format("{'mov':{'x':%d,'y':%d,'z':%d, 'a':%d,'mv':%d}}",rs.x, rs.y, rs.z, rotSteps, rawFeedrate), 10000);
            }
            else{         // Cartesian move with no rotation.  Feedrate is just the cartesian feedrate
                logger.trace(String.format("moveTo: Cartesian move, feedrate=%d steps/second",rawFeedrate));
                sendJsonCommand(String.format("{'mov':{'x':%d,'y':%d,'z':%d,'mv':%d}}",rs.x, rs.y, rs.z, rawFeedrate), 10000);
            }
        }
        else {
            if (moveRot){ // Rotation, no Cartesian move.  Feedrate is just the rotation feedrate
                setRotMotorEnable(true);
                logger.trace(String.format("moveTo: Rotation move, feedrate=%d steps/second",rawFeedrate));
                sendJsonCommand(String.format("{'mov':{'a':%d,'mv':%d}}",rotSteps, rawFeedrate), 10000);
            }
            else{         // No move, nothing to do
                logger.trace("moveTo: No move, nothing to do");
            }
        }
    }
    
    public void moveToFireStepKinematics(ReferenceHeadMountable hm, Location location, double speed)
            throws Exception {
        sendJsonCommand(String.format("{'mov':{'x':%f,'y':%f,'z':%f,'a':%f,'mv':%d}}", 
                -location.getX(), 
                -location.getY(), 
                location.getZ(),
                location.getRotation() * nozzleStepsPerDegree,
                rawFeedrate
                ), 10000);
    }
    
	@Override
	public void pick(ReferenceNozzle nozzle) throws Exception {
		setRotMotorEnable(true); // Enable the nozzle rotation
		// wait a tick for the motor to settle down from being enabled. it tends
		// to twitch a little when it's first enabled
		Thread.sleep(250);
		
		enableVacuumPump(true);  // Enable the pump
	}
	
	@Override
	public void place(ReferenceNozzle nozzle) throws Exception {
		enableVacuumPump(false);
		
		// TODO: this is a temporary hack which turns on a vacuum exhaust
		// solenoid for a short time to quickly vent the vacuum. This will need
		// to be changed to some other pin when we're actually using both
		// the dispenser and the nozzle at the same time.
        enableDispenser(true);
        Thread.sleep(250);
        enableDispenser(false);

        // disable the rotation motor last so that it doesn't twitch the part
        // out of place.
        setRotMotorEnable(false);
	}
	
	public synchronized void connect()
			throws Exception {
	    super.connect();
	
		/**
		 * Connection process notes:
		 * 
		 * On some platforms, as soon as we open the serial port it will reset
		 * FireStep and we'll start getting some data. On others, FireStep may
		 * already be running and we will get nothing on connect.
		 */
	    
        connectedVersion = "";
        connected = false;

        readerThread = new Thread(this);
		readerThread.start();
		
		// Flick DTR to reset the Arduino, if possible.
		serialPort.setDTR(true);
		Thread.sleep(100);
		serialPort.setDTR(false);
		
        // Wait up to 5 seconds for FireStep to say hi, and throw away the
		// results. Due to the startup EEPROM stuff this could include
		// any number of messages. We just want to ignore it all.
		try {
		    sendJsonCommand(null, 5000);
		}
		catch (Exception e) {
		    // We might receive trash, or nothing, so catch the potential exception.
		}
		
		// Now we want to synchronize with the flow control. We send junk that
		// we know that FireStep would never send and see if we see it in the
		// response. If we do we know that we are synced up to the current
		// command.
		boolean synced = false;
		for (int i = 0; !synced && i < 3; i++) {
		    try {
		        String key = "__" + i;
		        for (JsonObject o : sendJsonCommand(String.format("{'%s':''}", key), 5000, false)) {
		            if (o.get("r").getAsJsonObject().has(key)) {
		                synced = true;
		                break;
		            }
		        }
		    }
		    catch (Exception e) {
		    }
		}
		
        if (!synced)  {
            throw new Exception(
                String.format("Unable to receive connection response from FireStep. Check your port and baud rate, and that you are running at least version %f of FireStep", 
                        minimumRequiredVersion));
        }
	
        // Now send the version request. Any previous version response should
        // have been eaten up earlier so the next one we see should be valid.
        for (JsonObject o : sendJsonCommand("{'sysv':''}")) {
            JsonObject response = o.get("r").getAsJsonObject();
            if (response.has("sysv")) {
                connected = true;
                connectedVersion = response.get("sysv").getAsString();
            }
        }
		
        if (!connected)  {
            throw new Exception(
                String.format("Unable to receive connection response from FireStep. Check your port and baud rate, and that you are running at least version %f of FireStep", 
                        minimumRequiredVersion));
        }
        
		//TODO: Commenting this out for now. Will implement version checks once we get the prototoype working.
		//if (connectedVersion < minimumRequiredVersion) {
		//	throw new Error(String.format("This driver requires Marlin version %.2f or higher. You are running version %.2f", minimumRequiredVersion, connectedVersion));
		//}
		
	    //TODO: Allow configuration of modular tools 
		setXyzMotorEnable(false);    // Disable all motors
        setMotorDirection(true, false, false); // Set X/Y motors to normal and rotation to inverted.
		setHomingSpeed(200);				// Set the homing speed to something slower than default
		sendJsonCommand("{'ape':34}"); // Set the enable pin for axis 'a' to tool 4 (this is an ugly hack and should go away)
		// Turn off the stepper drivers
		setEnabled(false);
	}
	
	public synchronized void disconnect() {
		disconnectRequested = true;
		connected = false;
		
		try {
			if (readerThread != null && readerThread.isAlive()) {
				readerThread.join();
			}
		}
		catch (Exception e) {
			logger.error("disconnect()", e);
		}
		
		try {
		    super.disconnect();
	    }
	    catch (Exception e) {
	        logger.error("disconnect()", e);
	    }
		disconnectRequested = false;
	}
	
	/**
	 * Get the machine's current location according to FireStep.
	 * @return
	 * @throws Exception
	 */
	public Location getFireStepLocation() throws Exception {
	    if (useFireStepKinematics) {
            JsonObject o = sendJsonCommand("{'mpo':''}").get(0).get("r").getAsJsonObject().get("mpo").getAsJsonObject();
            // TODO: Rotation
            return new Location(LengthUnit.Millimeters, 
                    o.get("x").getAsDouble(),
                    o.get("y").getAsDouble(),
                    o.get("z").getAsDouble(),
                    0
                    );
	    }
	    else {
	        JsonObject o = sendJsonCommand("{'mpo':''}").get(0).get("r").getAsJsonObject().get("mpo").getAsJsonObject();
	        return parseLocationFromSteps(o, new String[] { "1", "2", "3" });
	    }
	}
	
	/**
	 * Set the internal state of the driver's location to the given location.
	 * @param location
	 * @throws Exception
	 */
	public void setLocation(Location location) throws Exception {
	    this.x = location.getX();
	    this.y = location.getY();
	    this.z = location.getZ();
	    ReferenceMachine machine = (ReferenceMachine) Configuration.get().getMachine();
	    ReferenceHead head = (ReferenceHead) machine.getHeads().get(0);
        machine.fireMachineHeadActivity(head);
	}
	
	/**
	 * Given a JsonObject containing step values for x, y and z, perform
	 * delta kinematics to determine the given cartesian location and return
	 * it. 
	 * @param o
	 * @return
	 * @throws Exception
	 */
	private Location parseLocationFromSteps(JsonObject o, String[] fields) throws Exception {
        int x = o.get(fields[0]).getAsInt();
        int y = o.get(fields[1]).getAsInt();
        int z = o.get(fields[2]).getAsInt();
        return deltaCalculator.getLocation(new RawStepTriplet(x, y, z));
	}
	
	public List<Location> probeCorners(ReferenceHeadMountable hm) throws Exception {
        Location startLocation = hm.getLocation();
        double distance = 85;
        List<Location> locations = new ArrayList<Location>();
        // front left
        locations.add(probePoint(hm, startLocation.derive(-distance, -distance, null, null)));
        // front right
        locations.add(probePoint(hm, startLocation.derive(distance, -distance, null, null)));
        // back right
        locations.add(probePoint(hm, startLocation.derive(distance, distance, null, null)));
        // back left
        locations.add(probePoint(hm, startLocation.derive(-distance, distance, null, null)));
        hm.moveTo(startLocation, 1.0);
        return locations;
    }

	public Location probePoint(ReferenceHeadMountable hm, Location startLocation)  throws Exception {
		hm.moveTo(startLocation, 1.0);
		sendJsonCommand("{'prb':''}", 30000);
		setLocation(getFireStepLocation());
		Location location = hm.getLocation();
		hm.moveTo(startLocation, 1.0);
		return location;
	}
	
    public double[][] probeGrid(ReferenceHeadMountable hm) throws Exception {
        logger.debug("Performing Z probe...");
        
        double DELTA_PROBABLE_RADIUS = 90.0; //214mm / 2 -10
        double LEFT_PROBE_BED_POSITION  = -DELTA_PROBABLE_RADIUS;
        double RIGHT_PROBE_BED_POSITION =  DELTA_PROBABLE_RADIUS;
        double BACK_PROBE_BED_POSITION  =  DELTA_PROBABLE_RADIUS;
        double FRONT_PROBE_BED_POSITION = -DELTA_PROBABLE_RADIUS;

        int ACCURATE_BED_LEVELING_POINTS = 7;       
        double ACCURATE_BED_LEVELING_GRID_Y = ((BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (ACCURATE_BED_LEVELING_POINTS - 1));
        double ACCURATE_BED_LEVELING_GRID_X = ((RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (ACCURATE_BED_LEVELING_POINTS - 1));

        double bed_level[][] = new double[ACCURATE_BED_LEVELING_POINTS][ACCURATE_BED_LEVELING_POINTS];
        
        // First, do a probe at 0,0 to determine the approximate bed height and then work
        // slightly above it.
        Location startLocation = probePoint(hm, hm.getLocation().derive(0d, 0d, null, null));
        startLocation = startLocation.add(new Location(LengthUnit.Millimeters, 0, 0, 20, 0));

        for (int yCount = 0; yCount < ACCURATE_BED_LEVELING_POINTS; yCount++) {
            double yProbe = FRONT_PROBE_BED_POSITION
                    + ACCURATE_BED_LEVELING_GRID_Y * yCount;
            int xStart, xStop, xInc;
            if ((yCount % 2) != 0) {
                xStart = 0;
                xStop = ACCURATE_BED_LEVELING_POINTS;
                xInc = 1;
            } else {
                xStart = ACCURATE_BED_LEVELING_POINTS - 1;
                xStop = -1;
                xInc = -1;
            }

            for (int xCount = xStart; xCount != xStop; xCount += xInc) {
                double xProbe = LEFT_PROBE_BED_POSITION + ACCURATE_BED_LEVELING_GRID_X * xCount;

                // Avoid probing the corners (outside the round or hexagon print surface) on a delta printer.
                double distance_from_center = Math.sqrt(xProbe * xProbe + yProbe * yProbe);
                if (distance_from_center <= DELTA_PROBABLE_RADIUS) {
                    // Now do the Z probe
                    Location probedLocation = new Location(LengthUnit.Millimeters, xProbe, yProbe, startLocation.getZ(), 0);
                    double measured_z = 0;
                    measured_z = probePoint(hm, probedLocation).getZ();
                    bed_level[xCount][yCount] = measured_z;
                }
                else {
                    bed_level[xCount][yCount] = Double.NaN;
                }
            }
        }
        hm.moveTo(startLocation, 1.0);
        return bed_level;
    }
    
    public Future<Void> barycentricCapture(final boolean unmappedOnly, FutureCallback<Void> callback) {
        final int numTries = 10;
        final double maxErrorDistance = 5;
        final double minErrorDistance = 0.1;
        final Map<Point, DomainAndRange> map = new LinkedHashMap<>();
        return Configuration.get().getMachine().submit(new Callable<Void>() {
            @Override
            public Void call() throws Exception {
                ReferenceCamera camera = (ReferenceCamera) Configuration.get().getMachine().getHeads().get(0).getCameras().get(0);
                Location startLocation = unmappedOnly ? barycentricCalibration.startLocation : camera.getLocation();

                Location unitsPerPixel = camera
                        .getUnitsPerPixel()
                        .convertToUnits(barycentricCalibration.gridCircleDiameter.getUnits());
                double uppMin = Math.min(unitsPerPixel.getX(), unitsPerPixel.getY());
                double uppMax = Math.max(unitsPerPixel.getX(), unitsPerPixel.getY());
                double uppAverage = (uppMin + uppMax) / 2;
                double radiusMin = barycentricCalibration
                        .gridCircleDiameter
                        .getValue() / uppMin / 2;
                double radiusMax = barycentricCalibration
                        .gridCircleDiameter
                        .getValue() / uppMax / 2;
                // Add 5% to each radius to account for mumble, mumble, mumble. Just trust me.
                radiusMin -= radiusMin * 0.05;
                radiusMax += radiusMax * 0.05;
                double minimumDistance = barycentricCalibration.gridMinimumPointSeparation.getValue() / uppAverage; 
                
                try {
                    int pointsChecked = 0;
                    List<Point> points = unmappedOnly ? 
                            barycentricCalibration.getUnmappedGridPoints() 
                            : barycentricCalibration.gridPoints;
                    for (Point point : points) {
                        if (Thread.currentThread().isInterrupted()) {
                            logger.info("Barycentric capture interrupted, cancelling.");
                        }
                        logger.info("Checking {} of {} points.", ++pointsChecked, points.size());
                        Location location = new Location(
                                barycentricCalibration.gridPointsUnits,
                                point.getX(), 
                                point.getY(), 
                                0, 
                                0);
                        location = startLocation.add(location.convertToUnits(startLocation.getUnits()));
                        
                        Location visionLocation = location;
                        boolean found = false;
                        for (int i = 0; i < numTries; i++) {
                            // move to where we expect to find the circle
                            camera.moveTo(visionLocation, 1.0);
                            
                            // wait a tick
                            Thread.sleep(1000);
                            
                            List<Location> circles = findCircles(camera, radiusMin, radiusMax, minimumDistance);
                            Location visionLocation2 = null;
                            if (!circles.isEmpty()) {
                                visionLocation2 = circles.get(0);
                            }
                            
                            // if nothing found, loop and try again
                            if (visionLocation2 == null) {
                                continue;
                            }
                            
                            // update units so we can do some unit independent math
                            visionLocation = visionLocation.convertToUnits(LengthUnit.Millimeters);
                            visionLocation2 = visionLocation2.convertToUnits(LengthUnit.Millimeters);
                            
                            // if the circle we found is more than 5mm from the point where
                            // it should be then we probably have a vision problem, so reset
                            // back to the original location and try again
                            if (Math.abs(visionLocation2.getLinearDistanceTo(location)) > maxErrorDistance) {
                                logger.warn("Got too far away from location, starting over.");
                                visionLocation = location;
                                continue;
                            }
                            
                            // See how far we moved since the last try
                            double distance = visionLocation2.getLinearDistanceTo(visionLocation);
                            distance = Math.abs(distance);
                            
                            // Update the running location with the new one
                            visionLocation = visionLocation2;
                            
                            // Zero out the rotation since circles don't rotate.
                            visionLocation = visionLocation.derive(null, null, null, 0d);
                            
                            // If we moved less than 0.1 to find the new location then we're probably
                            // just circling the ideal, so call it good
                            if (distance < minErrorDistance) {
                                logger.info("FOUND {} / {} / {} -> {} -> {} {}", new Object[] {
                                        visionLocation.subtract(location),
                                        distance,
                                        i + 1,
                                        location,
                                        visionLocation,
                                        location.subtract(startLocation)
                                });
                                found = true;
                                break;
                            }
                        }
                        if (!found) {
                            logger.warn("FAILED {} {}", location, location.subtract(startLocation));
                        }
                        else {
                            logger.debug("{} -> {} -> {}", location, visionLocation);
                            map.put(point, new DomainAndRange(location, visionLocation));
                        }
                    }
                    camera.moveTo(startLocation, 1.0);
                }
                catch (Exception e) {
                    e.printStackTrace();
                }
                if (!unmappedOnly) {
                    barycentricCalibration.getMappedGridPoints().clear();
                }
                barycentricCalibration.getMappedGridPoints().putAll(map);
                barycentricCalibration.startLocation = startLocation;
                return null;
            }
        }, callback);
    }
    
    /**
     * Attempt to determine which tool is installed using the bottom
     * vision camera.
     * @return
     * @throws Exception
     */
    public HeadMountable detectInstalledTool() throws Exception {
        Machine machine = Configuration.get().getMachine();
        Head head = machine.getHeads().get(0);
        Camera camera = machine.getCameras().get(0);
        Nozzle nozzle = head.getNozzles().get(0);
        PasteDispenser dispenser = head.getPasteDispensers().get(0);

        List<Location> locations;
        
        double nozzleRadius = 38.78037;
        double dispenserRadius = 17.0709154;
        
        // the paste dispenser is longer, so try that one first so
        // that we don't potentially collide it with the camera.
        MovableUtils.moveToLocationAtSafeZ(dispenser, camera.getLocation(), 1.0);
        Thread.sleep(1000);
        locations = findCircles(camera, dispenserRadius - 1, dispenserRadius + 1, 400);
        Location dispenserLocation = null;
        if (!locations.isEmpty()) {
            Location location = locations.get(0);
            if (location.getLinearDistanceTo(camera.getLocation()) < 10) {
                dispenserLocation = location;
            }
        }
        
        // now try the nozzle
        MovableUtils.moveToLocationAtSafeZ(nozzle, camera.getLocation(), 1.0);
        Thread.sleep(1000);
        locations = findCircles(camera, nozzleRadius - 1, nozzleRadius + 1, 400);
        Location nozzleLocation = null;
        if (!locations.isEmpty()) {
            Location location = locations.get(0);
            if (location.getLinearDistanceTo(camera.getLocation()) < 10) {
                nozzleLocation = location;
            }
        }
        
        if (nozzleLocation == null && dispenserLocation == null) {
            return null;
        }
        else if (nozzleLocation == null) {
            return dispenser;
        }
        else if (dispenserLocation == null) {
            return nozzle;
        }
        // if we got a hit for both then it's probably the nozzle because
        // the nozzle's ID is the same as the dispenser's OD, so sometimes
        // we get a dispenser hit on the nozzle.
        return nozzle;
    }
    
    private void updateToolOffsets(ReferenceHeadMountable hm) throws Exception {
        if (!(hm instanceof Nozzle) && !(hm instanceof PasteDispenser)) {
            throw new Exception("Can't check offsets for " + hm);
        }
        Camera camera = Configuration.get().getMachine().getCameras().get(0);
        
        logger.info("Finding offsets for " + hm);
        double radius;
        double nozzleRadius = 38.78037;
        double dispenserRadius = 17.0709154;
        if (hm instanceof Nozzle) {
            radius = nozzleRadius;
        }
        else {
            radius = dispenserRadius;
        }
        Location location = camera.getLocation();
        MovableUtils.moveToLocationAtSafeZ(hm, location, 1.0);
        for (int i = 0; i < 3; i++) {
            Thread.sleep(1000);
            List<Location> locations = findCircles(camera, radius - 2, radius + 2, 400);
            if (locations.isEmpty()) {
                throw new Exception("Unable to find candidate circle matches.");
            }
            Location visionLocation = locations.get(0);
            if (visionLocation.getLinearDistanceTo(camera.getLocation()) > 10) {
                throw new Exception("Found a candidate match but it's too far away. Likely false positive.");
            }
            Location offsets = camera.getLocation().subtract(visionLocation);
            location = location.add(offsets);
            hm.moveTo(location, 1.0);
        }
        Location offsets = camera.getLocation().subtract(location);
        hm.setHeadOffsets(hm.getHeadOffsets().add(offsets));
        MovableUtils.moveToLocationAtSafeZ(hm, camera.getLocation(), 1.0);
    }
    
    public void checkToolOffsets() throws Exception {
        MainFrame.machineControlsPanel.submitMachineTask(new Runnable() {
            @Override
            public void run() {
                try {
                    Machine machine = Configuration.get().getMachine();
                    Head head = machine.getHeads().get(0);
                    Camera camera = machine.getCameras().get(0);

                    ReferenceHeadMountable hm = (ReferenceHeadMountable) detectInstalledTool();
                    if (hm == null) {
                        System.out.println("No tool installed");
                    }
                    System.out.println("Finding offsets for " + hm);
                    double radius;
                    double nozzleRadius = 38.78037;
                    double dispenserRadius = 17.0709154;
                    if (hm instanceof Nozzle) {
                        radius = nozzleRadius;
                    }
                    else {
                        radius = dispenserRadius;
                    }
                    Location location = camera.getLocation();
                    for (int i = 0; i < 3; i++) {
                        MovableUtils.moveToLocationAtSafeZ(hm, location, 1.0);
                        Thread.sleep(1000);
                        List<Location> locations = findCircles(camera, radius - 2, radius + 2, 400);
                        if (locations.isEmpty()) {
                            System.out.println("no matches found");
                            return;
                        }
                        Location visionLocation = locations.get(0);
                        if (visionLocation.getLinearDistanceTo(camera.getLocation()) > 10) {
                            System.out.println("False positive");
                            return;
                        }
                        Location offsets = camera.getLocation().subtract(visionLocation);
                        location = location.add(offsets);
                    }
                    Location offsets = camera.getLocation().subtract(location);
                    hm.setHeadOffsets(hm.getHeadOffsets().add(offsets));
                    MovableUtils.moveToLocationAtSafeZ(hm, camera.getLocation(), 1.0);
                    
                }
                catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });
    }
    
    /**
     * Find circles in the camera view within the radius range. Circles are
     * returned in order of closest to furthest from the center. Normal location
     * objects are returned, except that the rotation field is set to the
     * circle's radius.
     * @param camera
     * @param minRadius
     * @param maxRadius
     * @return
     */
    private List<Location> findCircles(final Camera camera, double minRadius, double maxRadius, double minDistance) throws Exception {
        JsonArray pipeline = new JsonParser()
            .parse("[{'name': 's1','op': 'HoughCircles','houghcircles_minDist': 400.0,'show': true,'diamMin': 10.0,'diamMax': 90.0}]")
            .getAsJsonArray();
        JsonObject s1 = pipeline.get(0).getAsJsonObject();
        s1.addProperty("diamMin", minRadius * 2);
        s1.addProperty("diamMax", maxRadius * 2);
        s1.addProperty("houghcircles_minDist", minDistance);
        BufferedImage image = camera.capture();
        FireSightResult result = FireSight.fireSight(image, pipeline);
        List<Location> locations = new ArrayList<>();
        for (JsonElement e : result.model.get("s1").getAsJsonObject().get("circles").getAsJsonArray()) {
            JsonObject o = e.getAsJsonObject();
            double x = o.get("x").getAsDouble();
            double y = o.get("y").getAsDouble();
            double radius = o.get("radius").getAsDouble();
            Location location = VisionUtils.getPixelLocation(camera, x, y);
            location = location.derive(null, null, null, radius);
            locations.add(location); 
        }
        Collections.sort(locations, new Comparator<Location>() {
            public int compare(Location o1, Location o2) {
                Double o1d = camera.getLocation().getLinearDistanceTo(o1); 
                Double o2d = camera.getLocation().getLinearDistanceTo(o2);
                return o1d.compareTo(o2d);
            }
        });
        return locations;
    }
    
	private void setMotorDirection(boolean xyz, boolean rot, boolean enable) throws Exception {
	    logger.trace(String.format("%s%s Stepper motor Direction set to %s", xyz?"XYZ":"", rot?"A":"", enable?"enabled":"disabled" ));
	    sendFireStepConfig(xyz, rot, "dh", enable?"true":"false");
	}

	private void setXyzMotorEnable(boolean enable) throws Exception {
	    logger.trace(String.format("XYZ Stepper motor Enable set to %s", enable?"enabled":"disabled" ));
	    sendFireStepConfig(true, false, "en", enable?"true":"false");
	}
	
	private void setRotMotorEnable(boolean enable) throws Exception {
	    logger.trace(String.format("Rotation Stepper motor Enable set to %s", enable?"enabled":"disabled" ));
	    if (enable) {
	    	if (nozzleEnabled) {
	    		//Already enabled, nothing to do
	    	}
	    	else
	    	{
	    	    sendFireStepConfig(false, true, "en", "true"); //Enable power for XYZ stepper motors
				Thread.sleep(200,0);                  // Delay for a bit, wait for stepper motor coils to stabilize.
	    	}
	    }
	    else //if not enabled
	    {
	    	if (nozzleEnabled) {
	    	    sendFireStepConfig(false, true, "en", "false"); //Enable power for XYZ stepper motors
	    	}
	    	else
	    	{
	    		//Already disabled, nothing to do
	    	}
	    }
	    nozzleEnabled = enable; //Set state variable	    
	}

	private void setHomingSpeed(int delay) throws Exception {
	    // TODO: This no longer seems to work in FireStep.
//		sendJsonCommand(String.format("{'xsd':%d,'ysd':%d,'zsd':%d}",delay,delay,delay), 100);       // Search delay (think this is the homing speed)
	}
	

	private void enablePowerSupply(boolean enable) throws Exception {
	    logger.trace(String.format("FireStep: Power supply: %s", enable?"Turned ON":"Turned OFF" ));
		toggleDigitalPin(28,enable);
		powerSupplyOn = enable;
	}
	
	private void enableEndEffectorRingLight(boolean enable) throws Exception {
	    logger.trace(String.format("FireStep: End effector LED ring light: %s", enable?"Turned ON":"Turned OFF" ));
		toggleDigitalPin(5,enable);
	}
	
	private void enableUpLookingRingLight(boolean enable) throws Exception {
	    logger.trace(String.format("FireStep: Up-looking LED ring light: %s", enable?"Turned ON":"Turned OFF" ));
		toggleDigitalPin(4,enable);
	}
	
	private void enableVacuumPump(boolean enable) throws Exception {
	    logger.trace(String.format("FireStep: Vacuum pump: %s", enable?"Enabled":"Disabled" ));
		if (usePwmVacuum) {
		    setPwmPin(26,enable?pwmVacuumSetting:0);
		}
		else {
			toggleDigitalPin(26,enable); //Legacy support for older firestep users
		}
			
	}
	
	private void enableDispenser(boolean enable) throws Exception {
	    toggleDigitalPin(50, enable);
	}
	
	public void setVacuumPumpPwm(int value) {
		pwmVacuumSetting = value;
		//TODO: Exception if PWM level is outside of 0-255
	}

	public void toggleDigitalPin(int pin, boolean state) throws Exception {
	    logger.trace(String.format("FireStep: Toggling digital pin %d to %s", pin, state?"HIGH":"LOW" ));
        try {
			sendJsonCommand(String.format("{'iod%d':%s}", pin, state?"true":"false"));
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public void setPwmPin(int pin, int value) throws Exception {
	    logger.trace(String.format("FireStep: Setting PWM pin %d to %d", pin, value ));
        try {
        	//TODO: Throw an exception if value is not between 0 and 255.
			sendJsonCommand(String.format("{'ioa%d':%d}", pin, value));
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	private void sendFireStepConfig(boolean xyz, boolean rot, String param, String value) throws Exception {
	    if (xyz && rot) {
			sendJsonCommand(String.format("{'x%s':%s,'y%s':%s,'z%s':%s,'a%s':%s}",param,value,param,value,param,value,param,value));
	    }
	    else if (xyz) {
			sendJsonCommand(String.format("{'x%s':%s,'y%s':%s,'z%s':%s}",param,value,param,value,param,value));
	    }
	    else if (rot) {
			sendJsonCommand(String.format("{'a%s':%s}",param,value));
	    }
	}
	
    private void setFireStepKinematicsEnabled(boolean enabled) throws Exception {
        if (enabled) {
        	ReferenceNozzle nozzle = (ReferenceNozzle) Configuration
        			.get()
        			.getMachine()
        			.getHeads()
        			.get(0)
        			.getNozzles()
        			.get(0);
            sendJsonCommand("{'systo':1}");
            // I know these can all be sent in one go, but it makes the
            // code hard to read and modify. Feel free to change it if it
            // bugs you.
            sendJsonCommand(String.format("{'dimst':%d}", (int) deltaCalculator.getStepsPerMotorRotation()));
            sendJsonCommand(String.format("{'dimmi':%d}", (int) deltaCalculator.getMotorMicrosteps()));

            sendJsonCommand(String.format("{'dimgr1':%f}", deltaCalculator.getPulleyReductionX()));
            sendJsonCommand(String.format("{'dimgr2':%f}", deltaCalculator.getPulleyReductionY()));
            sendJsonCommand(String.format("{'dimgr3':%f}", deltaCalculator.getPulleyReductionZ()));
            
            sendJsonCommand(String.format("{'dime':%f}", deltaCalculator.getE()));
            sendJsonCommand(String.format("{'dimf':%f}", deltaCalculator.getF()));
            sendJsonCommand(String.format("{'dimre':%f}", deltaCalculator.getrE()));
            sendJsonCommand(String.format("{'dimrf':%f}", deltaCalculator.getrF()));
            sendJsonCommand(String.format("{'dimspr':%f}", 0f));
            sendJsonCommand(String.format("{'dimhzl':%f}", -10f));
            sendJsonCommand(String.format("{'syspb':%d}", 53));
            sendJsonCommand(String.format("{'syspu':%d}", 1));
            
            /*
             * When homing in MTO_FPD mode, FireStep requires that the home
             * step counts all be the same. We want to use different ones so
             * that we can zero the arms to horizontal, so we fake it by
             * sending the difference as latch backoff and then using the
             * min value when homing. The addition of 200 is just so that we
             * always have a minimum amount of latch backoff.             
             */
            RawStepTriplet hasc = deltaCalculator.getHomeRawSteps();
            int hascMin = Math.min(hasc.x, Math.min(hasc.y, hasc.z));
            sendJsonCommand(String.format("{'xlb':%d}", (int) (200 + hasc.x - hascMin)));
            sendJsonCommand(String.format("{'ylb':%d}", (int) (200 + hasc.y - hascMin)));
            sendJsonCommand(String.format("{'zlb':%d}", (int) (200 + hasc.z - hascMin)));
        }
        else {
            sendJsonCommand("{'systo':0}");
        }
    }
    
    public boolean isUseFireStepKinematics() {
        return useFireStepKinematics;
    }

    public void setUseFireStepKinematics(boolean useFireStepKinematics) {
        this.useFireStepKinematics = useFireStepKinematics;
    }

    public RotatableDeltaKinematicsCalculator getDeltaCalculator() {
        return deltaCalculator;
    }

    /**
     * Send a command and wait 5 seconds for a response. This is just a
     * shortcut for {@link #sendJsonCommand(String, long)} with a default
     * timeout.
     * @param command
     * @return
     * @throws Exception
     */
    public List<JsonObject> sendJsonCommand(String command) throws Exception {
        return sendJsonCommand(command, 5000);
    }
	
    public List<JsonObject> sendJsonCommand(String command, long timeout) throws Exception {
        return sendJsonCommand(command, timeout, true);
    }

        /**
	 * Send a command and wait timeout milliseconds for it to return.
	 * @param command
	 * @param timeout
	 * @return
	 * @throws Exception
	 */
    public List<JsonObject> sendJsonCommand(String command, long timeout, boolean checkStatus) throws Exception {
        List<JsonObject> responses = new ArrayList<>();
        
        // Read any responses that might be queued up so that when we wait
        // for a response to a command we actually wait for the one we expect.
        responseQueue.drainTo(responses);

        if (command != null) {
            // Replace single quotes with double quotes. This makes it easier to
            // write the commands in code without having to constantly escape
            // double quotes. We need the double quotes so it will parse
            // correctly.
            command = command.replaceAll("'", "\"");
            logger.trace("sendCommand({}, {})", command, timeout);
            output.write(command.getBytes());
            output.write("\n".getBytes());
        }
        
        // Wait up to timeout milliseconds for the a response to return from
        // the reader.
        JsonObject response = responseQueue.poll(timeout, TimeUnit.MILLISECONDS);
        if (response == null) {
            throw new Exception("Timeout waiting for response to " + command);
        }
        // And if we got one, add it to the list of responses we'll return.
        responses.add(response);
        
        // Read any additional responses that came in after the initial one.
        responseQueue.drainTo(responses);

        if (checkStatus) {
            // Check if any of the responses were failures.
            for (JsonObject o : responses) {
                if (o.has("s") && o.get("s").getAsInt() != 0) {
                    logger.error("{} => {}", command, responses);
                    throw new Exception("FireStep command failed " + o);
                }
            }
        }
        
        logger.debug("{} => {}", command, responses);
        return responses;
	}
	
	//Serial receive thread. Just reads lines and throws them in a queue.
	public void run() {
	    // Contains lines of multiline responses that are not yet complete.
	    StringBuffer lineBuffer = new StringBuffer();
		while (!disconnectRequested) {
	        String line;
	        try {
	            line = readLine();
	        }
	        catch (TimeoutException ex) {
	            continue;
	        }
	        catch (IOException e) {
	            logger.error("Read error", e);
	            return;
	        }
			logger.trace("'{}'", line);
            lineBuffer.append(line);
            // Look for the end of response token from FireStep
			if (line.endsWith("} ")) {
			    // If this is the end of a response we can package up
			    // everything that has come before it and parse it.
				try {
				    JsonObject o = parser
				            .parse(lineBuffer.toString())
				            .getAsJsonObject();
		            responseQueue.offer(o);
	                lineBuffer = new StringBuffer();
				}
				catch (Exception e) {
					logger.error("JSON parse error", e);
				}
			}
		}
	}
	
	public boolean isAutoUpdateToolOffsets() {
        return autoUpdateToolOffsets;
    }

    public void setAutoUpdateToolOffsets(boolean autoUpdateToolOffsets) {
        this.autoUpdateToolOffsets = autoUpdateToolOffsets;
    }

    @Override
	public Wizard getConfigurationWizard() {
		//return null;
	    return new FireStepDriverWizard(this);
	}
	@Override
	public String getPropertySheetHolderTitle() {
	    return getClass().getSimpleName();
	}
	
	@Override
	public PropertySheetHolder[] getChildPropertySheetHolders() {
	    // TODO Auto-generated method stub
	    return null;
	}
	
	@Override
	public Action[] getPropertySheetHolderActions() {
	    // TODO Auto-generated method stub
	    return null;
	}
	
	@Override
	public PropertySheet[] getPropertySheets() {
	    return new PropertySheet[] {
	            new PropertySheetWizardAdapter(getConfigurationWizard())
	    };
	}
	
	public BarycentricCalibration getBarycentricCalibration() {
        return barycentricCalibration;
    }

    public void setBarycentricCalibration(
            BarycentricCalibration barycentricCalibration) {
        this.barycentricCalibration = barycentricCalibration;
    }
    
    public CameraPoseCalibration getCameraPoseCalibration() {
        return cameraPoseCalibration;
    }

    public void setCameraPoseCalibration(CameraPoseCalibration cameraPoseCalibration) {
        this.cameraPoseCalibration = cameraPoseCalibration;
    }

    public static class BarycentricCalibration {
        private static final Logger logger = LoggerFactory.getLogger(BarycentricCalibration.class);
        
        /**
         * A default grid of 29x29 points with corners and hard to get to points excluded.
         */
        final private static String defaultGridPoints = "-80,-140:-70,-140:-60,-140:-50,-140:-40,-140:-30,-140:-20,-140:-10,-140:0,-140:10,-140:20,-140:30,-140:40,-140:50,-140:60,-140:70,-140:80,-140:-80,-130:-70,-130:-60,-130:-50,-130:-40,-130:-30,-130:-20,-130:-10,-130:0,-130:10,-130:20,-130:30,-130:40,-130:50,-130:60,-130:70,-130:80,-130:-90,-120:-80,-120:-70,-120:-60,-120:-50,-120:-40,-120:-30,-120:-20,-120:-10,-120:0,-120:10,-120:20,-120:30,-120:40,-120:50,-120:60,-120:70,-120:80,-120:90,-120:-100,-110:-90,-110:-80,-110:-70,-110:-60,-110:-50,-110:-40,-110:-30,-110:-20,-110:-10,-110:0,-110:10,-110:20,-110:30,-110:40,-110:50,-110:60,-110:70,-110:80,-110:90,-110:100,-110:-140,-100:-130,-100:-120,-100:-110,-100:-100,-100:-90,-100:-80,-100:-70,-100:-60,-100:-50,-100:-40,-100:-30,-100:-20,-100:-10,-100:0,-100:10,-100:20,-100:30,-100:40,-100:50,-100:60,-100:70,-100:80,-100:90,-100:100,-100:110,-100:120,-100:130,-100:140,-100:-140,-90:-130,-90:-120,-90:-110,-90:-100,-90:-90,-90:-80,-90:-70,-90:-60,-90:-50,-90:-40,-90:-30,-90:-20,-90:-10,-90:0,-90:10,-90:20,-90:30,-90:40,-90:50,-90:60,-90:70,-90:80,-90:90,-90:100,-90:110,-90:120,-90:130,-90:140,-90:-140,-80:-130,-80:-120,-80:-110,-80:-100,-80:-90,-80:-80,-80:-70,-80:-60,-80:-50,-80:-40,-80:-30,-80:-20,-80:-10,-80:0,-80:10,-80:20,-80:30,-80:40,-80:50,-80:60,-80:70,-80:80,-80:90,-80:100,-80:110,-80:120,-80:130,-80:140,-80:-140,-70:-130,-70:-120,-70:-110,-70:-100,-70:-90,-70:-80,-70:-70,-70:-60,-70:-50,-70:-40,-70:-30,-70:-20,-70:-10,-70:0,-70:10,-70:20,-70:30,-70:40,-70:50,-70:60,-70:70,-70:80,-70:90,-70:100,-70:110,-70:120,-70:130,-70:140,-70:-140,-60:-130,-60:-120,-60:-110,-60:-100,-60:-90,-60:-80,-60:-70,-60:-60,-60:-50,-60:-40,-60:-30,-60:-20,-60:-10,-60:0,-60:10,-60:20,-60:30,-60:40,-60:50,-60:60,-60:70,-60:80,-60:90,-60:100,-60:110,-60:120,-60:130,-60:140,-60:-140,-50:-130,-50:-120,-50:-110,-50:-100,-50:-90,-50:-80,-50:-70,-50:-60,-50:-50,-50:-40,-50:-30,-50:-20,-50:-10,-50:0,-50:10,-50:20,-50:30,-50:40,-50:50,-50:60,-50:70,-50:80,-50:90,-50:100,-50:110,-50:120,-50:130,-50:140,-50:-140,-40:-130,-40:-120,-40:-110,-40:-100,-40:-90,-40:-80,-40:-70,-40:-60,-40:-50,-40:-40,-40:-30,-40:-20,-40:-10,-40:0,-40:10,-40:20,-40:30,-40:40,-40:50,-40:60,-40:70,-40:80,-40:90,-40:100,-40:110,-40:120,-40:130,-40:140,-40:-140,-30:-130,-30:-120,-30:-110,-30:-100,-30:-90,-30:-80,-30:-70,-30:-60,-30:-50,-30:-40,-30:-30,-30:-20,-30:-10,-30:0,-30:10,-30:20,-30:30,-30:40,-30:50,-30:60,-30:70,-30:80,-30:90,-30:100,-30:110,-30:120,-30:130,-30:140,-30:-140,-20:-130,-20:-120,-20:-110,-20:-100,-20:-90,-20:-80,-20:-70,-20:-60,-20:-50,-20:-40,-20:-30,-20:-20,-20:-10,-20:0,-20:10,-20:20,-20:30,-20:40,-20:50,-20:60,-20:70,-20:80,-20:90,-20:100,-20:110,-20:120,-20:130,-20:140,-20:-140,-10:-130,-10:-120,-10:-110,-10:-100,-10:-90,-10:-80,-10:-70,-10:-60,-10:-50,-10:-40,-10:-30,-10:-20,-10:-10,-10:0,-10:10,-10:20,-10:30,-10:40,-10:50,-10:60,-10:70,-10:80,-10:90,-10:100,-10:110,-10:120,-10:130,-10:140,-10:-140,0:-130,0:-120,0:-110,0:-100,0:-90,0:-80,0:-70,0:-60,0:-50,0:-40,0:-30,0:-20,0:-10,0:0,0:10,0:20,0:30,0:40,0:50,0:60,0:70,0:80,0:90,0:100,0:110,0:120,0:130,0:140,0:-140,10:-130,10:-120,10:-110,10:-100,10:-90,10:-80,10:-70,10:-60,10:-50,10:-40,10:-30,10:-20,10:-10,10:0,10:10,10:20,10:30,10:40,10:50,10:60,10:70,10:80,10:90,10:100,10:110,10:120,10:130,10:140,10:-140,20:-130,20:-120,20:-110,20:-100,20:-90,20:-80,20:-70,20:-60,20:-50,20:-40,20:-30,20:-20,20:-10,20:0,20:10,20:20,20:30,20:40,20:50,20:60,20:70,20:80,20:90,20:100,20:110,20:120,20:130,20:140,20:-140,30:-130,30:-120,30:-110,30:-100,30:-90,30:-80,30:-70,30:-60,30:-50,30:-40,30:-30,30:-20,30:-10,30:0,30:10,30:20,30:30,30:40,30:50,30:60,30:70,30:80,30:90,30:100,30:110,30:120,30:130,30:140,30:-140,40:-130,40:-120,40:-110,40:-100,40:-90,40:-80,40:-70,40:-60,40:-50,40:-40,40:-30,40:-20,40:-10,40:0,40:10,40:20,40:30,40:40,40:50,40:60,40:70,40:80,40:90,40:100,40:110,40:120,40:130,40:140,40:-90,50:-80,50:-70,50:-60,50:-50,50:-40,50:-30,50:-20,50:-10,50:0,50:10,50:20,50:30,50:40,50:50,50:60,50:70,50:80,50:90,50:-90,60:-80,60:-70,60:-60,60:-50,60:-40,60:-30,60:-20,60:-10,60:0,60:10,60:20,60:30,60:40,60:50,60:60,60:70,60:80,60:90,60:-90,70:-80,70:-70,70:-60,70:-50,70:-40,70:-30,70:-20,70:-10,70:0,70:10,70:20,70:30,70:40,70:50,70:60,70:70,70:80,70:90,70:-90,80:-80,80:-70,80:-60,80:-50,80:-40,80:-30,80:-20,80:-10,80:0,80:10,80:20,80:30,80:40,80:50,80:60,80:70,80:80,80:90,80:-70,90:-60,90:-50,90:-40,90:-30,90:-20,90:-10,90:0,90:10,90:20,90:30,90:40,90:50,90:60,90:70,90:-60,100:-50,100:-40,100:-30,100:-20,100:-10,100:0,100:10,100:20,100:30,100:40,100:50,100:60,100:-60,110:-50,110:-40,110:-30,110:-20,110:-10,110:0,110:10,110:20,110:30,110:40,110:50,110:60,110:-60,120:-50,120:-40,120:-30,120:-20,120:-10,120:0,120:10,120:20,120:30,120:40,120:50,120:60,120:-60,130:-50,130:-40,130:-30,130:-20,130:-10,130:0,130:10,130:20,130:30,130:40,130:50,130:60,130:-60,140:-50,140:-40,140:-30,140:-20,140:-10,140:0,140:10,140:20,140:30,140:40,140:50,140:60,140";

        @Attribute
        private boolean enabled;
        
	    @Element
	    private Length gridCircleDiameter = new Length(3.5, LengthUnit.Millimeters);
	    
	    @Attribute
	    private LengthUnit gridPointsUnits = LengthUnit.Millimeters;
	    
	    @ElementList
	    private List<Point> gridPoints = new ArrayList<>();
	    
	    @Element
	    private Length gridMinimumPointSeparation = new Length(8, LengthUnit.Millimeters);
	    
	    @Element
	    private Location startLocation = new Location(LengthUnit.Millimeters);
	    
        @ElementMap(required=false)
        private Map<Point, DomainAndRange> mappedPoints = new LinkedHashMap<>();
        
        public BarycentricCalibration(boolean addDefaultGridPoints) {
            if (addDefaultGridPoints) {
                gridPoints.addAll(getDefaultGridPoints());
            }
        }
        
        public BarycentricCalibration() {
            this(false);
        }
        
        @Commit
        private void commit() {
            logger.info("{} unmapped points of {} total points.", getUnmappedGridPoints().size(), gridPoints.size());
        }
        
        private static List<Point> getDefaultGridPoints() {
            List<Point> gridPoints = new ArrayList<>();
            String[] points = defaultGridPoints.split(":");
            for (String point : points) {
                String[] coords = point.split(",");
                double x = Double.parseDouble(coords[0]);
                double y = Double.parseDouble(coords[1]);
                gridPoints.add(new Point(x, y));
            }
            return gridPoints;
        }
        
        public Map<Location, Location> getMap() {
            Map<Location, Location> map = new LinkedHashMap<Location, Location>();
            for (DomainAndRange o : mappedPoints.values()) {
                map.put(o.domain, o.range);
            }
            return map;
        }

	    public List<Point> getUnmappedGridPoints() {
	        List<Point> gridPoints = new ArrayList<>(this.gridPoints);
            Set<Point> mappedPoints = this.mappedPoints.keySet();
	        gridPoints.removeAll(mappedPoints);
	        return gridPoints;
	    }
	    
	    public Map<Point, DomainAndRange> getMappedGridPoints() {
	        return mappedPoints;
	    }

        public boolean isEnabled() {
            return enabled;
        }

        public void setEnabled(boolean enabled) {
            this.enabled = enabled;
        }

        public Length getGridCircleDiameter() {
            return gridCircleDiameter;
        }

        public void setGridCircleDiameter(Length gridCircleDiameter) {
            this.gridCircleDiameter = gridCircleDiameter;
        }

        public LengthUnit getGridPointsUnits() {
            return gridPointsUnits;
        }

        public void setGridPointsUnits(LengthUnit gridPointsUnits) {
            this.gridPointsUnits = gridPointsUnits;
        }

        public List<Point> getGridPoints() {
            return gridPoints;
        }

        public void setGridPoints(List<Point> gridPoints) {
            this.gridPoints = gridPoints;
        }

        public Length getGridMinimumPointSeparation() {
            return gridMinimumPointSeparation;
        }

        public void setGridMinimumPointSeparation(Length gridMinimumPointSeparation) {
            this.gridMinimumPointSeparation = gridMinimumPointSeparation;
        }
        
        public static class DomainAndRange {
            @Element
            Location domain;
            @Element
            Location range;
            
            public DomainAndRange() {
                
            }
            
            public DomainAndRange(Location domain, Location range) {
                this.domain = domain;
                this.range = range;
            }
        }
	}
    
    public static class CameraPoseCalibration {
        @Attribute(required=false)
        boolean enabled = false;
        
        @Element(required=false)
        Location top = new Location(LengthUnit.Millimeters);
        
        @Element(required=false)
        Location bottom = new Location(LengthUnit.Millimeters);

        public boolean isEnabled() {
            return enabled;
        }

        public void setEnabled(boolean enabled) {
            this.enabled = enabled;
        }

        public Location getTop() {
            return top;
        }

        public void setTop(Location top) {
            this.top = top;
        }

        public Location getBottom() {
            return bottom;
        }

        public void setBottom(Location bottom) {
            this.bottom = bottom;
        }
    }
}
