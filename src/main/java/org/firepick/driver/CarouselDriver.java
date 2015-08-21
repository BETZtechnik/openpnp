package org.firepick.driver;

import java.awt.image.BufferedImage;

import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.openpnp.spi.Camera;
import org.openpnp.spi.Nozzle;
import org.simpleframework.xml.Attribute;
import org.simpleframework.xml.Element;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.Result;
import com.google.zxing.ResultPoint;
import com.google.zxing.client.j2se.BufferedImageLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

public class CarouselDriver {
    @Attribute
    private int address = 0;
    
    @Element(required = false)
    private Location homeLocation = new Location(LengthUnit.Millimeters);
    
    private FireStepDriver machineDriver;
    
    private int selectedTray = 0;
    
    public void setMachineDriver(FireStepDriver machineDriver) {
        this.machineDriver = machineDriver;
    }
    
    public void home() throws Exception {
        // move to home location
        // spin the feeder until we find the fiducial
        // call that home and set the offsets
//        Camera camera = Configuration.get().getMachine().getHeads().get(0).getCameras().get(0);
//        MovableUtils.moveToLocationAtSafeZ(camera, homeLocation, 1.0);
        machineDriver.toggleDigitalPin(44, true);
    }
    
    public void selectTray(Nozzle nozzle, int tray) throws Exception {
        // assume tray 0 is position 0
        // never rotate more than 180 degrees
        // read location of A
        // turn on the rotation pin
        // move to where we need to be in A
        // turn off the rotation pin
        if (tray == selectedTray) {
            return;
        }
        double currentPosition = selectedTray * 45;
        double targetPosition = tray * 45;
        double deltaPosition = targetPosition - currentPosition;
        Location location = nozzle.getLocation();
        machineDriver.toggleDigitalPin(44, false);
        System.out.println(location);
        location = location.derive(null, null, null, location.getRotation() + deltaPosition);
        System.out.println(location);
        nozzle.moveTo(location, 1.0);
        machineDriver.toggleDigitalPin(44, true);
        selectedTray = tray;
    }
    
    private void centerQrCode(Camera camera) throws Exception {
        BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(
                new BufferedImageLuminanceSource(camera.capture())));
        Result result = new QRCodeReader().decode(binaryBitmap);
        System.out.println(result.getText());

        ResultPoint[] resultPoints = result.getResultPoints();
        // orders the points such that 0 and 2 are the most distant. these
        // are the corners of the QR code.
        ResultPoint.orderBestPatterns(resultPoints);
        // find the midpoint between 0 and 2 to determine the center of the
        // code.
        ResultPoint a = resultPoints[0];
        ResultPoint c = resultPoints[2];
        double midX = (a.getX() + c.getX()) / 2;
        double midY = (a.getY() + c.getY()) / 2;
        System.out.println(a);
        System.out.println(c);
        System.out.println(midX + "," + midY);
        Location offset = getPixelCenterOffsets(camera, (int) midX, (int) midY);
        System.out.println(offset);
        Location location = camera.getLocation().subtract(offset);
        System.out.println(location);
        camera.moveTo(location, 1.0);
    }
    
    private static Location getPixelCenterOffsets(Camera camera, int x, int y) {
        // match now contains the position, in pixels, from the top left corner
        // of the image to the top left corner of the match. We are interested in
        // knowing how far from the center of the image the center of the match is.
        BufferedImage image = camera.capture();
        double imageWidth = image.getWidth();
        double imageHeight = image.getHeight();

        // Calculate the difference between the center of the image to the
        // center of the match.
        double offsetX = (imageWidth / 2) - x;
        double offsetY = (imageHeight / 2) - y;

        // Invert the Y offset because images count top to bottom and the Y
        // axis of the machine counts bottom to top.
        offsetY *= -1;
        
        // And convert pixels to units
        Location unitsPerPixel = camera.getUnitsPerPixel();
        offsetX *= unitsPerPixel.getX();
        offsetY *= unitsPerPixel.getY();

        return new Location(camera.getUnitsPerPixel().getUnits(), offsetX, offsetY, 0, 0);
    }

    public int getAddress() {
        return address;
    }

    public void setAddress(int address) {
        this.address = address;
    }

    public Location getHomeLocation() {
        return homeLocation;
    }

    public void setHomeLocation(Location homeLocation) {
        this.homeLocation = homeLocation;
    }

    public int getSelectedTray() {
        return selectedTray;
    }

    public void setSelectedTray(int selectedTray) {
        this.selectedTray = selectedTray;
    }

    public FireStepDriver getMachineDriver() {
        return machineDriver;
    }
    
    
}
