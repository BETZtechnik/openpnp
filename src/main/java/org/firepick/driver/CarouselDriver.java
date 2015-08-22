package org.firepick.driver;

import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.openpnp.spi.Nozzle;
import org.simpleframework.xml.Attribute;
import org.simpleframework.xml.Element;

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
//        if (thread == null) {
//            new Thread(this).start();
//        }
    }
    
    public boolean selectTray(Nozzle nozzle, int tray) throws Exception {
        // assume tray 0 is position 0
        // never rotate more than 180 degrees
        // read location of A
        // turn on the rotation pin
        // move to where we need to be in A
        // turn off the rotation pin
        if (tray == selectedTray) {
            return false;
        }
        double currentPosition = selectedTray * 45;
        double targetPosition = tray * 45;
        double deltaPosition = targetPosition - currentPosition;
        Location location = nozzle.getLocation();
        machineDriver.toggleDigitalPin(44, false);
        location = location.derive(null, null, null, location.getRotation() + deltaPosition);
        nozzle.moveTo(location, 1.0);
        machineDriver.toggleDigitalPin(44, true);
        selectedTray = tray;
        return true;
    }
    
//    private void centerQrCode(Camera camera) throws Exception {
//        BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(
//                new BufferedImageLuminanceSource(camera.capture())));
//        Result result = new QRCodeReader().decode(binaryBitmap);
//        System.out.println(result.getText());
//
//        ResultPoint[] resultPoints = result.getResultPoints();
//        // orders the points such that 0 and 2 are the most distant. these
//        // are the corners of the QR code.
//        ResultPoint.orderBestPatterns(resultPoints);
//        // find the midpoint between 0 and 2 to determine the center of the
//        // code.
//        ResultPoint a = resultPoints[0];
//        ResultPoint c = resultPoints[2];
//        double midX = (a.getX() + c.getX()) / 2;
//        double midY = (a.getY() + c.getY()) / 2;
//        System.out.println(a);
//        System.out.println(c);
//        System.out.println(midX + "," + midY);
//        Location offset = getPixelCenterOffsets(camera, (int) midX, (int) midY);
//        System.out.println(offset);
//        Location location = camera.getLocation().subtract(offset);
//        System.out.println(location);
//        camera.moveTo(location, 1.0);
//    }
    
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
