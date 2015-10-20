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

import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.opencv.core.MatOfPoint;
import org.opencv.core.RotatedRect;
import org.openpnp.gui.MainFrame;
import org.openpnp.gui.components.CameraView;
import org.openpnp.gui.support.Wizard;
import org.openpnp.model.Board;
import org.openpnp.model.BoardLocation;
import org.openpnp.model.Configuration;
import org.openpnp.model.Length;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.openpnp.model.Part;
import org.openpnp.model.Placement;
import org.openpnp.spi.Camera;
import org.openpnp.spi.Camera.Looking;
import org.openpnp.spi.Feeder;
import org.openpnp.spi.Head;
import org.openpnp.spi.Machine;
import org.openpnp.spi.Nozzle;
import org.openpnp.spi.NozzleTip;
import org.openpnp.spi.VisionProvider;
import org.openpnp.spi.base.AbstractJobProcessor;
import org.openpnp.util.MovableUtils;
import org.openpnp.util.Utils2D;
import org.openpnp.util.VisionUtils;
import org.openpnp.vision.FiducialLocator;
import org.openpnp.vision.FluentCv;
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
		
        fireDetailedStatusUpdated(String.format("Prepare bottom vision."));        
        
        if (!shouldJobProcessingContinue()) {
            return;
        }

        try {
            prepareBottomVision();
        }
        catch (Exception e) {
            fireJobEncounteredError(JobError.MachineMovementError, e.getMessage());
            return;
        }
		
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
	    return performBottomVision(part, nozzle);
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
	
	private Map<Length, BufferedImage> bottomVisionBackgroundImages = new HashMap<>();
	/**
	 * Move the Nozzle to the camera and takes a picture which is stored
	 * as the background image for further bottom vision ops.
	 */
	private void prepareBottomVision() throws Exception {
		bottomVisionBackgroundImages.clear();
		
		Nozzle nozzle = Configuration.get().getMachine().getDefaultHead().getDefaultNozzle();
    	Camera camera = Configuration.get().getMachine().getCameras().get(0);
    	
        if (camera.getLooking() != Looking.Up) {
            throw new Exception("Bottom vision only implemented for Up looking cameras");
        }
        
        // figure out all the unique part heights we need to deal with
        Set<Length> heights = new HashSet<Length>();
        for (BoardLocation boardLocation : job.getBoardLocations()) {
        	Board board = boardLocation.getBoard();
        	for (Placement placement : board.getPlacements()) {
        		if (placement.getType() != Placement.Type.Place || placement.getSide() != boardLocation.getSide()) {
        			continue;
        		}
        		Part part = placement.getPart();
        		heights.add(part.getHeight());
        	}
        }
        
        logger.info("Capturing {} backgrounds.", heights.size());
        Location originalLocation = nozzle.getLocation();
        MovableUtils.moveToLocationAtSafeZ(nozzle, camera.getLocation(), 1.0);
        for (Length height : heights) {
        	Location heightLocation = new Location(height.getUnits(), 0, 0, height.getValue(), 0);
        	Location location = camera.getLocation().add(heightLocation);
        	nozzle.moveTo(location, 1.0);
            bottomVisionBackgroundImages.put(height, camera.settleAndCapture());
        }
        nozzle.moveToSafeZ(1.0);
	}
	
    private Location performBottomVision(Part part, Nozzle nozzle)
            throws Exception {
    	Camera camera = Configuration.get().getMachine().getCameras().get(0);
    	
        if (camera.getLooking() != Looking.Up) {
            throw new Exception("Bottom vision only implemented for Up looking cameras");
        }

        // Create a location that is the Camera's X, Y, it's Z + part height
        // and a rotation of 0.
        Location startLocation = camera.getLocation();
        Length partHeight = part.getHeight();
        Location partHeightLocation = new Location(partHeight.getUnits(), 0, 0, partHeight.getValue(), 0);
        startLocation = startLocation
        		.add(partHeightLocation)
        		.derive(null, null, null, 0d);

        MovableUtils.moveToLocationAtSafeZ(nozzle, startLocation, 1.0);
        
        BufferedImage backgroundImage = bottomVisionBackgroundImages.get(part.getHeight());
        
        File debugDir = new File("/Users/jason/Desktop/debug/" + System.currentTimeMillis());
        debugDir.mkdirs();
        
        for (int i = 0; i < 6; i++) {
        	File backgroundFile = new File(debugDir, i + "_background.png");
        	File foregroundFile = new File(debugDir, i + "_foreground.png");
        	File absDiffFile = new File(debugDir, i + "_asdiff.png");
        	File processedFile = new File(debugDir, i + "_processed.png");
        	
            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            List<RotatedRect> rects = new ArrayList<RotatedRect>();
            BufferedImage filteredImage = new FluentCv()
            	.setCamera(camera)
            	
            	.toMat(backgroundImage)
            	.write(backgroundFile)
            	.toGray()
            	.blurGaussian(3, "background")
            	
            	.settleAndCapture("original")
            	.write(foregroundFile)
            	.toGray()
            	.blurGaussian(3)
            	
            	.absDiff("background")
            	.write(absDiffFile)
            	
    			.blurGaussian(13)
    			.findEdgesRobertsCross()
    			.threshold(30)
    			.findContours(contours)
    			.recall("original")
    			.drawContours(contours, null, 1)
    			.getContourMaxRects(contours, rects)
    			.drawRects(rects, null, 2)
            	.write(processedFile)
            	.toBufferedImage();
            
            CameraView cameraView = MainFrame.mainFrame.cameraPanel.getCameraView(camera);
            cameraView.showFilteredImage(filteredImage, 3000);
            
            RotatedRect rect = rects.get(0);
            System.out.println(rect);
            		
            // Create the offsets object. This is the physical distance from
            // the center of the camera to the located part.
            Location offsets = VisionUtils.getPixelCenterOffsets(
                    camera,
                    rect.center.x, 
                    rect.center.y);

            // We assume that the part is never picked more than 45º rotated
            // so if OpenCV tells us it's rotated more than 45º we correct
            // it. This seems to happen quite a bit when the angle of rotation
            // is close to 0.
            double angle = rect.angle;
            if (Math.abs(angle) > 45) {
                if (angle < 0) {
                    angle += 90;
                }
                else {
                    angle -= 90;
                }
            }
            // Set the angle on the offsets.
            offsets = offsets.derive(null, null, null, angle);
            System.out.println("offsets " + offsets);
            
            // Move the nozzle so that the part is oriented correctly over the
            // camera.
            Location location = nozzle
                    .getLocation()
                    .subtractWithRotation(offsets);
            nozzle.moveTo(location, 1.0);
        }
        
//        for (int i = 0; i < 3; i++) {
//            // Settle
//            Thread.sleep(500);
//            // Grab an image.
//            BufferedImage image = camera.capture();
//            
//            // perform the vision op
//            FireSightResult result = FireSight.fireSight(image, "firesight/bottomVision.json");
//            List<RotatedRect> rects = FireSight.parseRotatedRects(
//                    result.model.get("minAreaRect").getAsJsonObject().get("rects").getAsJsonArray());
//            RotatedRect rect = rects.get(0);
//            
//            // Create the offsets object. This is the physical distance from
//            // the center of the camera to the located part.
//            Location offsets = VisionUtils.getPixelCenterOffsets(
//                    camera,
//                    rect.center.x, 
//                    rect.center.y);
//
//            // We assume that the part is never picked more than 45º rotated
//            // so if OpenCV tells us it's rotated more than 45º we correct
//            // it. This seems to happen quite a bit when the angle of rotation
//            // is close to 0.
//            double angle = rect.angle;
//            if (Math.abs(angle) > 45) {
//                if (angle < 0) {
//                    angle += 90;
//                }
//                else {
//                    angle -= 90;
//                }
//            }
//            // Set the angle on the offsets.
//            offsets = offsets.derive(null, null, null, -angle);
//            
//            // Move the nozzle so that the part is oriented correctly over the
//            // camera.
//            Location location = nozzle
//                    .getLocation()
//                    .subtractWithRotation(offsets);
//            nozzle.moveTo(location, 1.0);
//        }
//        
//        Location offsets = camera
//                .getLocation()
//                .subtractWithRotation(nozzle.getLocation());
//        
//        logger.debug("Final bottom vision offsets {}", offsets);
        
        // Return to Safe-Z just to be safe.
        nozzle.moveToSafeZ(1.0);
        return new Location(LengthUnit.Millimeters, 0, 0, 0, 0);
    } 
	
    @Override
    public Wizard getConfigurationWizard() {
        return null;
    }
}
