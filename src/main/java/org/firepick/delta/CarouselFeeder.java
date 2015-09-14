package org.firepick.delta;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import javax.swing.Action;

import org.firepick.delta.FireSight.FireSightResult;
import org.firepick.driver.CarouselDriver;
import org.firepick.driver.FireStepDriver;
import org.firepick.driver.wizards.CarouselFeederWizard;
import org.opencv.core.RotatedRect;
import org.opencv.core.Size;
import org.openpnp.gui.support.PropertySheetWizardAdapter;
import org.openpnp.gui.support.Wizard;
import org.openpnp.machine.reference.ReferenceFeeder;
import org.openpnp.machine.reference.ReferenceMachine;
import org.openpnp.model.Configuration;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.openpnp.spi.Camera;
import org.openpnp.spi.Nozzle;
import org.openpnp.spi.PropertySheetHolder;
import org.openpnp.util.MovableUtils;
import org.openpnp.util.VisionUtils;
import org.simpleframework.xml.Attribute;

public class CarouselFeeder extends ReferenceFeeder {
    @Attribute
    private int address = 0;
    
    @Attribute
    private int tray = 0;
    
    @Attribute
    private double partVisionWidth;
    
    @Attribute
    private double partVisionHeight;
    
    private Location pickLocation;
    
    @Override
    public boolean canFeedToNozzle(Nozzle nozzle) {
        return true;
    }

    @Override
    public Location getPickLocation() throws Exception {
        return pickLocation;
    }

    @Override
    public void feed(Nozzle nozzle) throws Exception {
        FireStepDriver machineDriver = (FireStepDriver) ((ReferenceMachine) Configuration.get().getMachine()).getDriver();
        CarouselDriver carouselDriver = machineDriver.getCarouselDriver(address);
        
        // select the correct tray
        if (carouselDriver.selectTray(nozzle, tray)) {
            Thread.sleep(1000);
        }
        
        // move the camera over the locating point and capture an image
        Camera camera = nozzle.getHead().getCameras().get(0);
        MovableUtils.moveToLocationAtSafeZ(camera, location, 1.0);

        // find the matching result closest to the camera
        // move to it
        // repeat
        for (int i = 0; i < 3; i++) {
            Thread.sleep(500);
            BufferedImage image = camera.capture();
            FireSightResult result = FireSight.fireSight(image, "multiPartDetect.json");
            List<RotatedRect> rects = FireSight
                    .parseRotatedRects(result.model.get("filtered")
                            .getAsJsonObject().get("rects").getAsJsonArray());
            RotatedRect bestMatch = findClosestMatch(rects, camera);
            // RotatedRect returns angles from 0 to -90, no matter which
            // size of the rectangle is the larger. So as the long side
            // rotates through 90 degrees the angle is reset. We want
            // to normalize that so we offset the angle by 90 degrees if
            // the height is the longer side.
            if (bestMatch.size.width < bestMatch.size.height) {
                bestMatch.angle -= 90;
            }
            bestMatch.angle = Math.abs(bestMatch.angle);
            Location l = VisionUtils.getPixelCenterOffsets(
                    camera, 
                    bestMatch.center.x, 
                    bestMatch.center.y);
            l = camera.getLocation().subtract(l);
            l = l.derive(null, null, null, bestMatch.angle);
            camera.moveTo(l, 1.0);
            pickLocation = l;
        }
    }
    
    /**
     * Filter invalid results and then sort by distance from center.
     * @param rects
     * @return
     */
    private RotatedRect findClosestMatch(List<RotatedRect> rects, final Camera camera) throws Exception {
        ArrayList<RotatedRect> filtered = new ArrayList<RotatedRect>();
        // filter out any that aren't the right width and height
        for (RotatedRect rect : rects) {
            Size s = rect.size;
            // TODO: How do we determine what the range is?
            double 
                majorMin = partVisionWidth - 4, 
                majorMax = partVisionWidth + 4, 
                minorMin = partVisionHeight - 4, 
                minorMax = partVisionHeight + 4;
            double major = Math.max(s.width,  s.height);
            double minor = Math.min(s.width, s.height);
            if (major >= majorMin && major <= majorMax && minor >= minorMin && minor <= minorMax) {
                filtered.add(rect);
            }
        }
        if (filtered.size() < 1) {
            throw new Exception("No rectangle matches found.");
        }
        Collections.sort(filtered, new Comparator<RotatedRect>() {
            @Override
            public int compare(RotatedRect o1, RotatedRect o2) {
                Location origin = new Location(LengthUnit.Millimeters);
                Double o1Dist = VisionUtils.getPixelCenterOffsets(camera, (int) o1.center.x, (int) o1.center.y).getLinearDistanceTo(origin); 
                Double o2Dist = VisionUtils.getPixelCenterOffsets(camera, (int) o2.center.x, (int) o2.center.y).getLinearDistanceTo(origin);
                return o1Dist.compareTo(o2Dist);
            }
        });
        return filtered.get(0);
    }
    
    public void selectTray() throws Exception {
        FireStepDriver machineDriver = (FireStepDriver) ((ReferenceMachine) Configuration.get().getMachine()).getDriver();
        CarouselDriver carouselDriver = machineDriver.getCarouselDriver(address);
        Nozzle nozzle = Configuration.get().getMachine().getHeads().get(0).getNozzles().get(0);
        carouselDriver.selectTray(nozzle, tray);
    }
    
    public int getAddress() {
        return address;
    }

    public void setAddress(int address) {
        this.address = address;
    }

    public int getTray() {
        return tray;
    }

    public void setTray(int tray) {
        this.tray = tray;
    }
    
    public double getPartVisionWidth() {
        return partVisionWidth;
    }

    public void setPartVisionWidth(double partVisionWidth) {
        this.partVisionWidth = partVisionWidth;
    }

    public double getPartVisionHeight() {
        return partVisionHeight;
    }

    public void setPartVisionHeight(double partVisionHeight) {
        this.partVisionHeight = partVisionHeight;
    }

    @Override
    public Wizard getConfigurationWizard() {
        return new CarouselFeederWizard(this);
    }

    @Override
    public String getPropertySheetHolderTitle() {
        return getClass().getSimpleName() + " " + getName();
    }

    @Override
    public PropertySheetHolder[] getChildPropertySheetHolders() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public PropertySheet[] getPropertySheets() {
        return new PropertySheet[] {
                new PropertySheetWizardAdapter(getConfigurationWizard())
        };
    }

    @Override
    public Action[] getPropertySheetHolderActions() {
        // TODO Auto-generated method stub
        return null;
    }      
}
