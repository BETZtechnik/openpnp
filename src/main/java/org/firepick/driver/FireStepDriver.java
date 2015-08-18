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

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.TimeoutException;

import javax.swing.Action;

import org.firepick.driver.wizards.FireStepDriverWizard;
import org.firepick.gfilter.GCoordinate;
import org.firepick.gfilter.MappedPointFilter;
import org.firepick.kinematics.RotatableDeltaKinematicsCalculator;
import org.firepick.model.AngleTriplet;
import org.firepick.model.RawStepTriplet;
import org.openpnp.gui.support.PropertySheetWizardAdapter;
import org.openpnp.gui.support.Wizard;
import org.openpnp.machine.reference.ReferenceActuator;
import org.openpnp.machine.reference.ReferenceHead;
import org.openpnp.machine.reference.ReferenceHeadMountable;
import org.openpnp.machine.reference.ReferenceMachine;
import org.openpnp.machine.reference.ReferenceNozzle;
import org.openpnp.machine.reference.driver.AbstractSerialPortDriver;
import org.openpnp.model.Configuration;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.openpnp.spi.PropertySheetHolder;
import org.simpleframework.xml.Attribute;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;


public class FireStepDriver extends AbstractSerialPortDriver implements Runnable {
	private static final Logger logger = LoggerFactory.getLogger(FireStepDriver.class);
	private static final double minimumRequiredVersion = 1.0;
	
	// NOTE: This is ignored out because FireStep doesn't use feed rates per se.. it just does everything rather quickly and smoothly.
	@Attribute
	private double feedRateMmPerMinute;
	
    @Attribute(required=false)
    private double xPlusScale = 1.025, xMinusScale = 1.025, yPlusScale = 1.04, yMinusScale = 1.02;
    
    @Attribute(required=false)
    private boolean useFireStepKinematics = false;
    
	//@Attribute
	private double nozzleStepsPerDegree =  8.888888888;
	private boolean nozzleEnabled = false;
	private boolean powerSupplyOn = false;
	private RotatableDeltaKinematicsCalculator deltaCalc = new RotatableDeltaKinematicsCalculator();

    private int rawFeedrate = 12800; //12800 is FireStep's default feedrate
	private double x, y, z, c;
	private Thread readerThread;
	private boolean disconnectRequested;
	private Object commandLock = new Object();
	private boolean connected;
	private String connectedVersion;
	private Queue<String> responseQueue = new ConcurrentLinkedQueue<String>();
	private JsonParser parser = new JsonParser();
	
	@Attribute(required=false)
	private boolean useGfilter = false;
	
	private MappedPointFilter gFilter;
	
	@Override
	public void setEnabled(boolean enabled) throws Exception {
	    if (enabled) {
	        if (useGfilter) {
	            File file = new File(Configuration.get().getConfigurationDirectory(), "gfilter.json");
	            gFilter = new ConcreteMappedPointFilter(new FileReader(file));
	        }
	        else {
	            gFilter = null;
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
	        enablePowerSupply(true);              		// Turn power supply ON
	        if (powerSupplyOn)					  		// Exception should catch but guard just in case
	        {
				Thread.sleep(500,0);                	// Delay for a bit, wait for power supply to stabilize.
				setXyzMotorEnable(true);				// Enable power for XYZ stepper motors
	    		enableEndEffectorRingLight(true); 		// Turn off down-looking LED ring light
				Thread.sleep(50,0);                 	// Delay for a bit, wait for power supply to stabilize.
		        home(null);                         	// home the machine
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
			        enableVacuumPump(false);           	// Turn the vacuum pump OFF
					setXyzMotorEnable(false);  			// Disable power for XYZ stepper motors
			        enablePowerSupply(false);          	// Turn off the power supply
	    		}
	    	}
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
	public void home(ReferenceHead head) throws Exception {
	    if (useFireStepKinematics) {
            RawStepTriplet rs = deltaCalc.getHomePosRaw();
            sendJsonCommand(String.format("{'hom':{'x':%d,'y':%d,'z':%d}}", rs.x, rs.y, rs.z), 10000);
            setLocation(deltaCalc.getHomePosCartesian());
	    }
	    else {
	        RawStepTriplet rs = deltaCalc.getHomePosRaw();
	        sendJsonCommand(String.format("{'hom':{'x':%d,'y':%d,'z':%d}}", rs.x, rs.y, rs.z), 10000);
	        setLocation(getFireStepLocation());
	    }
	}
	
	@Override
	public Location getLocation(ReferenceHeadMountable hm) {
		//TODO: Request raw step positions from FireStep, do forward delta kinematics, throw exception if they don't match this class's Cartesian pos.
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
	    location = location.subtract(hm.getHeadOffsets());
	    location = location.convertToUnits(LengthUnit.Millimeters);
	    // Update the target location to handle any NaNs that were supplied
	    location = location.derive(
                Double.isNaN(location.getX()) ? this.x : location.getX(),
                Double.isNaN(location.getY()) ? this.y : location.getY(),
                Double.isNaN(location.getZ()) ? this.z : location.getZ(),
                Double.isNaN(location.getRotation()) ? this.c : location.getRotation());
        double xScale = location.getX() < 0 ? xMinusScale : xPlusScale;
        double yScale = location.getY() < 0 ? yMinusScale : yPlusScale;
        Location scaledLocation = location.multiply(xScale, yScale, 1.0, 1.0);
        if (useGfilter) {
            GCoordinate coord = new GCoordinate(scaledLocation.getX(), scaledLocation.getY(), scaledLocation.getZ());
            GCoordinate mappedCoord = gFilter.interpolate(coord);
            logger.debug("gFilter mapped: {} -> {} -> {}", new Object[] { scaledLocation, coord, mappedCoord });
//            scaledLocation = scaledLocation.derive(mappedCoord.getX(), mappedCoord.getY(), mappedCoord.getZ(), null);
            scaledLocation = scaledLocation.derive(mappedCoord.getX(), mappedCoord.getY(), null, null);
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
            logger.debug(String.format("moveTo Cartesian: X: %.3f, Y: %.3f, Z: %.3f",location.getX(), location.getY(),location.getZ() ));
            
            // Calculate delta kinematics (returns angles)
            AngleTriplet angles = deltaCalc.calculateDelta(location);
            logger.debug(String.format("moveTo Delta: X: %.3f, Y: %.3f, Z: %.3f",angles.x, angles.y,angles.z ));
            
            // Convert angles into raw steps
            rs = deltaCalc.getRawSteps(angles);
            logger.debug(String.format("moveTo RawSteps: X: %d, Y: %d, Z: %d",rs.x, rs.y,rs.z ));
        }
        
        
        // Get feedrate in raw steps
        // Note that speed is defined by (maximum feed rate * speed) where speed is greater than 0 and typically less than or equal to 1. 
        // A speed of 0 means to move at the minimum possible speed.
        //TODO: Set feedrate based in raw steps, based off of 'feedRateMmPerMinute' and 'speed'
        // 'mv' is maximum velocity (pulses/second), and the default is 12800.

        rawFeedrate = (int)((double)rawFeedrate * speed); //Multiply rawFeedrate by speed, which should be 0 to 1
        if (moveXyz){
            if (moveRot){ // Cartesian move with rotation.  Feedrate is (TBD)
                logger.debug(String.format("moveTo: Cartesian move with rotation, feedrate=%d steps/second",rawFeedrate));
                setRotMotorEnable(true);
                sendJsonCommand(String.format("{'mov':{'x':%d,'y':%d,'z':%d, 'a':%d,'mv':%d}}",rs.x, rs.y, rs.z, rotSteps, rawFeedrate), 10000);
            }
            else{         // Cartesian move with no rotation.  Feedrate is just the cartesian feedrate
                logger.debug(String.format("moveTo: Cartesian move, feedrate=%d steps/second",rawFeedrate));
                sendJsonCommand(String.format("{'mov':{'x':%d,'y':%d,'z':%d,'mv':%d}}",rs.x, rs.y, rs.z, rawFeedrate), 10000);
            }
        }
        else {
            if (moveRot){ // Rotation, no Cartesian move.  Feedrate is just the rotation feedrate
                setRotMotorEnable(true);
                logger.debug(String.format("moveTo: Rotation move, feedrate=%d steps/second",rawFeedrate));
                sendJsonCommand(String.format("{'mov':{'a':%d,'mv':%d}}",rotSteps, rawFeedrate), 10000);
            }
            else{         // No move, nothing to do
                logger.debug("moveTo: No move, nothing to do");
            }
        }
    }
    
    public void moveToFireStepKinematics(ReferenceHeadMountable hm, Location location, double speed)
            throws Exception {
        sendJsonCommand(String.format("{'mov':{'x':%f,'y':%f,'z':%f}}", 
                -location.getX(), 
                -location.getY(), 
                location.getZ()
                ), 10000);
    }
    
    public void moveToAngles(double x, double y, double z) throws Exception {
        AngleTriplet angles = new AngleTriplet(x, y, z);
        RawStepTriplet steps = deltaCalc.getRawSteps(angles);
        sendJsonCommand(String.format("{'mov':{'x':%d,'y':%d,'z':%d}}", 
                steps.x, 
                steps.y, 
                steps.z
                ), 10000);
        setLocation(getFireStepLocation());
    }
	
	@Override
	public void pick(ReferenceNozzle nozzle) throws Exception {
		setRotMotorEnable(true); // Enable the nozzle rotation
		enableVacuumPump(true);  // Enable the pump
	}
	
	@Override
	public void place(ReferenceNozzle nozzle) throws Exception {
		enableVacuumPump(false);
		setRotMotorEnable(false);
	}
	
	public synchronized void connect()
			throws Exception {
	    super.connect();
	
		/**
		 * Connection process notes:
		 * 
		 * On some platforms, as soon as we open the serial port it will reset
		 * Grbl and we'll start getting some data. On others, Grbl may already
		 * be running and we will get nothing on connect.
		 */
		
		List<String> responses;
		synchronized (commandLock) {
			// Start the reader thread with the commandLock held. This will
			// keep the thread from quickly parsing any responses messages
			// and notifying before we get a change to wait.
			readerThread = new Thread(this);
			readerThread.start();
			// Wait up to 3 seconds for FireStep to say Hi
			// If we get anything at this point it will have been the settings
			// dump that is sent after reset.
			responses = sendCommand(null, 3000);
		}
	
		connectedVersion = "";
		connected = true;
		processStatusResponses(responses);
	
		for (int i = 0; i < 5 && !connected; i++) {
			sendJsonCommand("{'sys':''}",100);
		}
		
	if (!connected)  {
			throw new Error(
				String.format("Unable to receive connection response from FireStep. Check your port and baud rate, and that you are running at least version %f of Marlin", 
						minimumRequiredVersion));
		}
		
		//TODO: Commenting this out for now. Will implement version checks once we get the prototoype working.
		//if (connectedVersion < minimumRequiredVersion) {
		//	throw new Error(String.format("This driver requires Marlin version %.2f or higher. You are running version %.2f", minimumRequiredVersion, connectedVersion));
		//}
		
	    //TODO: Allow configuration of modular tools 
		setXyzMotorEnable(false);    // Disable all motors
        setFireStepKinematicsEnabled(useFireStepKinematics);
        setMotorDirection(true, false, false); // Set X/Y motors to normal and rotation to inverted.
		setHomingSpeed(200);				// Set the homing speed to something slower than default
		sendJsonCommand("{'ape':34}", 100); // Set the enable pin for axis 'a' to tool 4 (this is an ugly hack and should go away)
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
	
	private void setFireStepKinematicsEnabled(boolean enabled) throws Exception {
	    if (enabled) {
	        throw new Exception("Currently disabled, needs to be updated to handle independent gear ratios.");
//	        sendJsonCommand("{'systo':1}", 3000);
//	        sendJsonCommand(String.format("{ 'dim' : { 'e' : %f, 'f' : %f, 'gr' : %f, 'ha1' : %f, 'ha2' : %f, 'ha3' : %f, 'mi' : %d, 're' : %f, 'rf' : %f, 'st' : %d, 'zo' : %f}}", 
//	                deltaCalc.getE(),
//	                deltaCalc.getF(),
//	                deltaCalc.getGr(),
//	                deltaCalc.getHa1(),
//	                deltaCalc.getHa2(),
//	                deltaCalc.getHa3(),
//	                (int) deltaCalc.getMi(),
//	                deltaCalc.getRe(),
//	                deltaCalc.getRf(),
//	                (int) deltaCalc.getSt(),
//	                -deltaCalc.getZo()
//	                ), 3000);
	    }
	    else {
	        sendJsonCommand("{'systo':0}", 3000);
	    }
	}
	
	public JsonArray doNativeHexZprobe() throws Exception {
	    ReferenceMachine machine = (ReferenceMachine) Configuration.get().getMachine();
	    ReferenceHead head = (ReferenceHead) machine.getHeads().get(0);
        ReferenceNozzle nozzle = (ReferenceNozzle) head.getNozzles().get(0);
        
	    home(head);
	    moveTo(nozzle, new Location(LengthUnit.Millimeters), 1.0);

	    if (!useFireStepKinematics) {
	        setFireStepKinematicsEnabled(true);
	    }
	    
        sendJsonCommand("{'prbz':''}", 10000);

        sendJsonCommand("{'mov':{'rz':10,'a':0,'d':50}}", 10000);
        sendJsonCommand("{'prbz':''}", 10000);
        
        sendJsonCommand("{'mov':{'rz':10,'a':60,'d':50}}", 10000);
        sendJsonCommand("{'prbz':''}", 10000);
        
        sendJsonCommand("{'mov':{'rz':10,'a':120,'d':50}}", 10000);
        sendJsonCommand("{'prbz':''}", 10000);
        
        sendJsonCommand("{'mov':{'rz':10,'a':180,'d':50}}", 10000);
        sendJsonCommand("{'prbz':''}", 10000);
        
        sendJsonCommand("{'mov':{'rz':10,'a':240,'d':50}}", 10000);
        sendJsonCommand("{'prbz':''}", 10000);
        
        sendJsonCommand("{'mov':{'rz':10,'a':300,'d':50}}", 10000);
        sendJsonCommand("{'prbz':''}", 10000);
        
        sendJsonCommand("{'mov':{'rz':10,'a':0,'d':0}}", 10000);
        sendJsonCommand("{'prb':''}", 10000);

        sendJsonCommand("{'movz':0}", 10000);
        List<JsonObject> responses = sendJsonCommand("{'dimpd':''}", 10000);
        
        if (!useFireStepKinematics) {
            setFireStepKinematicsEnabled(false);
        }
        
        return responses.get(0).get("r").getAsJsonObject().get("dimpd").getAsJsonArray();
	}
	
	/**
	 * Get the machine's current location according to FireStep.
	 * @return
	 * @throws Exception
	 */
	public Location getFireStepLocation() throws Exception {
	    JsonObject o = sendJsonCommand("{'mpo':''}", 1000).get(0).get("r").getAsJsonObject().get("mpo").getAsJsonObject();
	    return parseLocationFromSteps(o, new String[] { "1", "2", "3" });
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
        return deltaCalc.getLocation(new RawStepTriplet(x, y, z));
	}
	
	public List<Location> doZprobeHex(ReferenceHeadMountable hm) throws Exception {
	    List<Location> locations = new ArrayList<Location>();
	    double radius = 50;
        locations.add(doZprobePoint(hm, new Location(LengthUnit.Millimeters, 0, 0, 0, 0)));
        locations.add(doZprobePoint(hm, new Location(LengthUnit.Millimeters, 0, radius, 0, 0)));
        locations.add(doZprobePoint(hm, new Location(LengthUnit.Millimeters, 0, radius, 0, 0).rotateXy(60)));
        locations.add(doZprobePoint(hm, new Location(LengthUnit.Millimeters, 0, radius, 0, 0).rotateXy(120)));
        locations.add(doZprobePoint(hm, new Location(LengthUnit.Millimeters, 0, radius, 0, 0).rotateXy(180)));
        locations.add(doZprobePoint(hm, new Location(LengthUnit.Millimeters, 0, radius, 0, 0).rotateXy(240)));
        locations.add(doZprobePoint(hm, new Location(LengthUnit.Millimeters, 0, radius, 0, 0).rotateXy(300)));
        locations.add(doZprobePoint(hm, new Location(LengthUnit.Millimeters, 0, 0, 0, 0)));
        return locations;
	}

    public List<Location> doZprobeCorners(ReferenceHeadMountable hm) throws Exception {
        double distance = 85;
        List<Location> locations = new ArrayList<Location>();
        // front left
        locations.add(doZprobePoint(hm, new Location(LengthUnit.Millimeters, -distance, -distance, 0, 0)));
        // front right
        locations.add(doZprobePoint(hm, new Location(LengthUnit.Millimeters, distance, -distance, 0, 0)));
        // back right
        locations.add(doZprobePoint(hm, new Location(LengthUnit.Millimeters, distance, distance, 0, 0)));
        // back left
        locations.add(doZprobePoint(hm, new Location(LengthUnit.Millimeters, -distance, distance, 0, 0)));
        hm.moveTo(new Location(LengthUnit.Millimeters, 0, 0, 0, 0), 1.0);
        return locations;
    }

	public Location doZprobePoint(ReferenceHeadMountable hm, Location startPoint)  throws Exception {
		this.moveTo(hm, startPoint, 1.0);
		
		//Determine point "below" the current target point.  Use inverse kinematics to get a point at Z=-20
		Location targetPoint =  new Location(LengthUnit.Millimeters, startPoint.getX(), startPoint.getY(), -100, 0);
		AngleTriplet at = deltaCalc.calculateDelta(targetPoint);
		RawStepTriplet raw = deltaCalc.getRawSteps(at);
		logger.debug(String.format("Do Z probe point : Location X=%.2f, Y=%.2f, Z=%.2f.",targetPoint.getX(), targetPoint.getY(), targetPoint.getZ() ));
		logger.debug(String.format("Do Z probe point : Raw angle %d, %d, %d.", raw.x, raw.y, raw.z ));
		
		int probePin = 6; //TODO: Make this configurable in EMC02 class
		//this.moveTo(hm,targetPoint, 1.0); //this is just for debugging, it's mutually exclusive with the prb command below...
		String prbStr = new String(String.format("{'prb':{'x':%d,'y':%d,'z':%d,'pn':%d}}",raw.x, raw.y, raw.z, probePin));
		List<JsonObject> responses = sendJsonCommand( prbStr, 5000 );
		logger.debug("Do Z probe point : Returning..");
		JsonObject o = responses.get(0).getAsJsonObject().get("r").getAsJsonObject().get("prb").getAsJsonObject();
		Location location = parseLocationFromSteps(o, new String[] { "x", "y", "z" });
		setLocation(getFireStepLocation());
        return location;
	}
	
    public double[][] doZProbeDetailed(ReferenceHeadMountable hm) throws Exception {
        logger.debug("Performing Z probe...");
        
        double DELTA_PROBABLE_RADIUS = 90.0; //214mm / 2 -10
        double LEFT_PROBE_BED_POSITION  = -DELTA_PROBABLE_RADIUS;
        double RIGHT_PROBE_BED_POSITION =  DELTA_PROBABLE_RADIUS;
        double BACK_PROBE_BED_POSITION  =  DELTA_PROBABLE_RADIUS;
        double FRONT_PROBE_BED_POSITION = -DELTA_PROBABLE_RADIUS;

        int ACCURATE_BED_LEVELING_POINTS = 9;       
        double ACCURATE_BED_LEVELING_GRID_Y = ((BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (ACCURATE_BED_LEVELING_POINTS - 1));
        double ACCURATE_BED_LEVELING_GRID_X = ((RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (ACCURATE_BED_LEVELING_POINTS - 1));

        double bed_level[][] = new double[ACCURATE_BED_LEVELING_POINTS][ACCURATE_BED_LEVELING_POINTS];
        
        // First, do a probe at 0,0 to determine the approximate bed height and then work
        // slightly above it.
        Location startLocation = doZprobePoint(hm, new Location(LengthUnit.Millimeters, 0, 0, 0, 0));
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
                    measured_z = doZprobePoint(hm, probedLocation).getZ();
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
    
	private void setMotorDirection(boolean xyz, boolean rot, boolean enable) throws Exception {
	    logger.debug(String.format("%s%s Stepper motor Direction set to %s", xyz?"XYZ":"", rot?"A":"", enable?"enabled":"disabled" ));
	    sendFireStepConfig(xyz, rot, "dh", enable?"true":"false");
	}

	private void setXyzMotorEnable(boolean enable) throws Exception {
	    logger.debug(String.format("XYZ Stepper motor Enable set to %s", enable?"enabled":"disabled" ));
	    sendFireStepConfig(true, false, "en", enable?"true":"false");
	}
	
	private void setRotMotorEnable(boolean enable) throws Exception {
	    logger.debug(String.format("Rotation Stepper motor Enable set to %s", enable?"enabled":"disabled" ));
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
		sendJsonCommand(String.format("{'xsd':%d,'ysd':%d,'zsd':%d}",delay,delay,delay), 100);       // Search delay (think this is the homing speed)
	}
	

	private void enablePowerSupply(boolean enable) throws Exception {
	    logger.debug(String.format("FireStep: Power supply: %s", enable?"Turned ON":"Turned OFF" ));
		toggleDigitalPin(28,enable);
		powerSupplyOn = enable;
	}
	
	private void enableEndEffectorRingLight(boolean enable) throws Exception {
	    logger.debug(String.format("FireStep: End effector LED ring light: %s", enable?"Turned ON":"Turned OFF" ));
		toggleDigitalPin(4,enable);
	}
	
	private void enableUpLookingRingLight(boolean enable) throws Exception {
	    logger.debug(String.format("FireStep: Up-looking LED ring light: %s", enable?"Turned ON":"Turned OFF" ));
		toggleDigitalPin(5,enable);
	}
	
	private void enableVacuumPump(boolean enable) throws Exception {
	    logger.debug(String.format("FireStep: Vacuum pump: %s", enable?"Enabled":"Disabled" ));
		toggleDigitalPin(26,enable);
	}

	private void toggleDigitalPin(int pin, boolean state) throws Exception {
	    logger.debug(String.format("FireStep: Toggling digital pin %d to %s", pin, state?"HIGH":"LOW" ));
        try {
			sendJsonCommand(String.format("{'iod%d':%s}", pin, state?"true":"false"),100);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	private void sendFireStepConfig(boolean xyz, boolean rot, String param, String value) throws Exception {
	    if (xyz && rot) {
			sendJsonCommand(String.format("{'x%s':%s,'y%s':%s,'z%s':%s,'a%s':%s}",param,value,param,value,param,value,param,value), 100);
	    }
	    else if (xyz) {
			sendJsonCommand(String.format("{'x%s':%s,'y%s':%s,'z%s':%s}",param,value,param,value,param,value), 100);
	    }
	    else if (rot) {
			sendJsonCommand(String.format("{'a%s':%s}",param,value), 100);
	    }
	}
	
	private List<JsonObject> processStatusResponses(List<String> responses) {
	    List<JsonObject> objects = new ArrayList<JsonObject>();
		for (String response : responses) 
		{
			if (response.startsWith("FireStep")) {
				logger.debug("echo: " + response);
				String[] versionComponents = response.split(" ");
				connectedVersion = versionComponents[1];
				connected = true;
				logger.debug(String.format("Connected to FireStep Version: %s", connectedVersion));
			}
			else {
			    JsonObject o = (JsonObject) parser.parse(response);
			    objects.add(o);
			}
		}
		return objects;
	}
	
	public List<JsonObject> sendJsonCommand(String command, long timeout) throws Exception {
		List<String> responses = sendCommand(command.replaceAll("'", "\""), timeout);
		return processStatusResponses(responses);
	}
	
	private List<String> sendCommand(String command, long timeout) throws Exception {
		synchronized (commandLock) {
			if (command != null) {
				logger.debug("sendCommand({}, {})", command, timeout);
				output.write(command.getBytes());
				output.write("\n".getBytes());
			}
			if (timeout == -1) {
				commandLock.wait();
			}
			else {
				commandLock.wait(timeout);
			}
		}
		List<String> responses = drainResponseQueue();
		return responses;
	}
	
	//Serial receive thread
	public void run() {
		while (!disconnectRequested) {
	        String line;
	        try {
	            line = readLine().trim();
	        }
	        catch (TimeoutException ex) {
	            continue;
	        }
	        catch (IOException e) {
	            logger.error("Read error", e);
	            return;
	        }
	        line = line.trim();
			logger.debug(line);
			responseQueue.offer(line);
			//if (line.equals("ok") || line.startsWith("error: ")) {
			if (line.isEmpty() == false) {
				// This is the end of processing for a command
				synchronized (commandLock) {
					commandLock.notify();
				}
			}
		}
	}
	
	private List<String> drainResponseQueue() {
		List<String> responses = new ArrayList<String>();
		String response;
		while ((response = responseQueue.poll()) != null) {
			responses.add(response);
		}
		return responses;
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
}
