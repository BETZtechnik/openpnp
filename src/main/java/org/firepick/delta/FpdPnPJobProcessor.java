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

package org.firepick.delta;

import org.openpnp.gui.support.Wizard;
import org.openpnp.model.Board;
import org.openpnp.model.BoardLocation;
import org.openpnp.model.Configuration;
import org.openpnp.model.Location;
import org.openpnp.model.Part;
import org.openpnp.model.Placement;
import org.openpnp.spi.Camera;
import org.openpnp.spi.Feeder;
import org.openpnp.spi.Head;
import org.openpnp.spi.Machine;
import org.openpnp.spi.Nozzle;
import org.openpnp.spi.NozzleTip;
import org.openpnp.spi.VisionProvider;
import org.openpnp.spi.base.AbstractJobProcessor;
import org.openpnp.util.Utils2D;
import org.openpnp.vision.FiducialLocator;
import org.simpleframework.xml.Attribute;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class FpdPnPJobProcessor extends AbstractJobProcessor {
	private static final Logger logger = LoggerFactory.getLogger(FpdPnPJobProcessor.class);
	
	@Attribute(required=false)
	String dummy;
	
	@Override
    public void run() {
		state = JobState.Running;
		fireJobStateChanged();
		
		Machine machine = Configuration.get().getMachine();
		
		for (Head head : machine.getHeads()) {
			fireDetailedStatusUpdated(String.format("Move head %s to Safe-Z.", head.getName()));		
	
			if (!shouldJobProcessingContinue()) {
				return;
			}
	
			try {
				head.moveToSafeZ(1.0);
			}
			catch (Exception e) {
				fireJobEncounteredError(JobError.MachineMovementError, e.getMessage());
				return;
			}
		}
		
        fireDetailedStatusUpdated(String.format("Check fiducials."));        
        
        if (!shouldJobProcessingContinue()) {
            return;
        }

        try {
            checkFiducials();
        }
        catch (Exception e) {
            fireJobEncounteredError(JobError.MachineMovementError, e.getMessage());
            return;
        }
		
		Head head = machine.getHeads().get(0);
		Nozzle nozzle = head.getNozzles().get(0);
		NozzleTip nozzleTip = nozzle.getNozzleTip();
		
		for (BoardLocation boardLocation : job.getBoardLocations()) {
		    Board board = boardLocation.getBoard();
		    for (Placement placement : board.getPlacements()) {
		        if (placement.getSide() != boardLocation.getSide()) {
		            continue;
		        }
		        
		        if (placement.getType() != Placement.Type.Place) {
		            continue;
		        }
		        
                Part part = placement.getPart();

                firePartProcessingStarted(boardLocation, placement);

                if (!nozzle.getNozzleTip().canHandle(part)) {
                    fireJobEncounteredError(JobError.PickError, "Selected nozzle tip is not compatible with part");
                    return;
                }
                
                if (!pick(nozzle, boardLocation, placement)) {
                    return;
                }
                
                fireDetailedStatusUpdated(String.format("Perform bottom vision"));      
                
                if (!shouldJobProcessingContinue()) {
                    return;
                }
                
                Location bottomVisionOffsets;
                try {
                    bottomVisionOffsets = performBottomVision(machine, part, nozzle);
                }
                catch (Exception e) {
                    fireJobEncounteredError(JobError.PartError, e.getMessage());
                    return;
                }
                
                Location placementLocation = placement.getLocation();
                if (bottomVisionOffsets != null) {
                    placementLocation = placementLocation.subtractWithRotation(bottomVisionOffsets);
                }
                placementLocation = Utils2D.calculateBoardPlacementLocation(
                        boardLocation.getLocation(), 
                        boardLocation.getSide(),
                        placementLocation);

                // Update the placementLocation with the proper Z value. This is
                // the distance to the top of the board plus the height of 
                // the part.
                Location boardLocationLocation = boardLocation
                        .getLocation()
                        .convertToUnits(placementLocation.getUnits());
                double partHeight = part
                        .getHeight()
                        .convertToUnits(placementLocation.getUnits())
                        .getValue();
                placementLocation = placementLocation
                        .derive(null, null, boardLocationLocation.getZ() + partHeight, null);
                
                if (!place(nozzle, boardLocation, placementLocation, placement)) {
                    return;
                }
		    }
		}
		
		fireDetailedStatusUpdated("Job complete.");
		
		state = JobState.Stopped;
		fireJobStateChanged();
	}

    // TODO: Should not bail if there are no fids on the board. Figure out
    // the UI for that.
    protected void checkFiducials() throws Exception {
        for (BoardLocation boardLocation : job.getBoardLocations()) {
            if (!boardLocation.isCheckFiducials()) {
                continue;
            }
            Location location = FiducialLocator.locateBoard(boardLocation);
            boardLocation.setLocation(location);
        }
    }
	
	protected Location performBottomVision(Machine machine, Part part, Nozzle nozzle) throws Exception {
	    // TODO: I think this stuff actually belongs in VisionProvider but
	    // I have not yet fully thought through the API.
	    
	    // Find the first fixed camera
	    if (machine.getCameras().isEmpty()) {
	        // TODO: return null for now to indicate that no vision was
	        // calculated. In the future we may want this to be based on
	        // configuration.
	        return null;
	    }
	    Camera camera = machine.getCameras().get(0);
	    
	    // Get it's vision provider
	    VisionProvider vp = camera.getVisionProvider();
	    if (vp == null) {
            // TODO: return null for now to indicate that no vision was
            // calculated. In the future we may want this to be based on
            // configuration.
            return null;
	    }
	    
	    // Perform the operation. Note that similar to feeding and nozzle
	    // tip changing, it is up to the VisionProvider to move the camera
	    // and nozzle to where it needs to be.
	    return vp.getPartBottomOffsets(part, nozzle);
	}
	
	protected Feeder feed(Nozzle nozzle, Part part) throws Exception {
        for (Feeder feeder : Configuration.get().getMachine().getFeeders()) {
            if (!shouldJobProcessingContinue()) {
                return null;
            }
            if (feeder.getPart() == part && feeder.isEnabled()) {
                try {
                    fireDetailedStatusUpdated(String.format("Request part feed from feeder %s.", feeder.getName()));
                    feeder.feed(nozzle);
                    return feeder;
                }
                catch (Exception e) {
                    logger.info("Feed on {} failed: {}", feeder.getName(), e.getMessage());
                }
            }
        }
        throw new Exception("No feeders available to feed " + part.getId());
    }
    
	protected boolean pick(Nozzle nozzle, BoardLocation boardLocation, Placement placement) {
	    Part part = placement.getPart();
        fireDetailedStatusUpdated(String.format("Move nozzle %s to Safe-Z at (%s).", nozzle.getName(), nozzle.getLocation()));        

        if (!shouldJobProcessingContinue()) {
            return false;
        }

        try {
            nozzle.moveToSafeZ(1.0);
        }
        catch (Exception e) {
            fireJobEncounteredError(JobError.MachineMovementError, e.getMessage());
            return false;
        }

        if (!shouldJobProcessingContinue()) {
            return false;
        }

        // Request that a Feeder feeds the part. Will attempt to feed from
        // multiple Feeders until one feeds without error.
        Feeder feeder = null;
        while (feeder == null) {
            if (!shouldJobProcessingContinue()) {
                return false;
            }
            try {
                feeder = feed(nozzle, part);
            }
            catch (Exception e) {
                fireJobEncounteredError(JobError.FeederError, e.getMessage());
            }
        }
        
        if (!shouldJobProcessingContinue()) {
            return false;
        }

        // Now that the Feeder has done it's feed operation we can get
        // the pick location from it.
        Location pickLocation;
        try {
            pickLocation = feeder.getPickLocation();
        }
        catch (Exception e) {
            fireJobEncounteredError(JobError.FeederError, e.getMessage());
            return false;
        }

        fireDetailedStatusUpdated(String.format("Move to safe Z at (%s).", nozzle.getLocation()));
        
        if (!shouldJobProcessingContinue()) {
            return false;
        }

        try {
            nozzle.moveToSafeZ(1.0);
        }
        catch (Exception e) {
            fireJobEncounteredError(JobError.MachineMovementError, e.getMessage());
            return false;
        }

        fireDetailedStatusUpdated(String.format("Move to pick location, safe Z at (%s).", pickLocation));

        if (!shouldJobProcessingContinue()) {
            return false;
        }
        
        // Move the Nozzle to the pick Location at safe Z
        try {
            nozzle.moveTo(pickLocation.derive(null, null, Double.NaN, null), 1.0);
        }
        catch (Exception e) {
            fireJobEncounteredError(JobError.MachineMovementError, e.getMessage());
            return false;
        }

        fireDetailedStatusUpdated(String.format("Move to pick location Z at (%s).", pickLocation));

        if (!shouldJobProcessingContinue()) {
            return false;
        }

        // Move the Nozzle to the pick Location 
        try {
            nozzle.moveTo(pickLocation, 1.0);
        }
        catch (Exception e) {
            fireJobEncounteredError(JobError.MachineMovementError, e.getMessage());
            return false;
        }

        fireDetailedStatusUpdated(String.format("Request part pick at (%s).", pickLocation));

        if (!shouldJobProcessingContinue()) {
            return false;
        }
        
        // Pick the part
        try {
            // TODO design a way for the head/feeder to indicate that the part
            // failed to pick, use the delegate to notify and potentially retry
            // We now have the delegate for this, just need to use it and 
            // implement the logic for it's potential responses
            nozzle.pick();
        }
        catch (Exception e) {
            fireJobEncounteredError(JobError.PickError, e.getMessage());
            return false;
        }
        
        firePartPicked(boardLocation, placement);

        fireDetailedStatusUpdated(String.format("Move to safe Z at (%s).", nozzle.getLocation()));

        if (!shouldJobProcessingContinue()) {
            return false;
        }

        try {
            nozzle.moveToSafeZ(1.0);
        }
        catch (Exception e) {
            fireJobEncounteredError(JobError.MachineMovementError, e.getMessage());
            return false;
        }
        
        return true;
	}
	
	protected boolean place(Nozzle nozzle, BoardLocation bl, Location placementLocation, Placement placement) {
        fireDetailedStatusUpdated(String.format("Move to placement location, safe Z at (%s).", placementLocation));

        if (!shouldJobProcessingContinue()) {
            return false;
        }

        // Move the nozzle to the placement Location at safe Z
        try {
            nozzle.moveTo(placementLocation.derive(null, null, Double.NaN, null), 1.0);
        }
        catch (Exception e) {
            fireJobEncounteredError(JobError.MachineMovementError, e.getMessage());
            return false;
        }

        fireDetailedStatusUpdated(String.format("Move to placement location Z at (%s).", placementLocation));

        if (!shouldJobProcessingContinue()) {
            return false;
        }

        // Lower the nozzle.
        try {
            nozzle.moveTo(placementLocation, 1.0);
        }
        catch (Exception e) {
            fireJobEncounteredError(JobError.MachineMovementError, e.getMessage());
            return false;
        }

        fireDetailedStatusUpdated(String.format("Request part place. at (X %2.3f, Y %2.3f, Z %2.3f, C %2.3f).", 
                placementLocation.getX(), 
                placementLocation.getY(), 
                placementLocation.getZ(), 
                placementLocation.getRotation()));

        if (!shouldJobProcessingContinue()) {
            return false;
        }
        
        // Place the part
        try {
            nozzle.place();
        }
        catch (Exception e) {
            fireJobEncounteredError(JobError.PlaceError, e.getMessage());
            return false;
        }
        
        firePartPlaced(bl, placement);
        
        fireDetailedStatusUpdated(String.format("Move to safe Z at (%s).", nozzle.getLocation()));      

        if (!shouldJobProcessingContinue()) {
            return false;
        }

        // Return to Safe-Z above the board. 
        try {
            nozzle.moveToSafeZ(1.0);
        }
        catch (Exception e) {
            fireJobEncounteredError(JobError.MachineMovementError, e.getMessage());
            return false;
        }
        
        firePartProcessingComplete(bl, placement);
        
        return true;
	}
	
    @Override
    public Wizard getConfigurationWizard() {
        return null;
    }
}
