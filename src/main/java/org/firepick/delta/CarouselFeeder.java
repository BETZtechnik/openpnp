package org.firepick.delta;

import javax.swing.Action;

import org.firepick.driver.CarouselDriver;
import org.firepick.driver.FireStepDriver;
import org.firepick.driver.wizards.CarouselFeederWizard;
import org.openpnp.gui.support.PropertySheetWizardAdapter;
import org.openpnp.gui.support.Wizard;
import org.openpnp.machine.reference.ReferenceFeeder;
import org.openpnp.machine.reference.ReferenceMachine;
import org.openpnp.model.Configuration;
import org.openpnp.model.Location;
import org.openpnp.spi.Nozzle;
import org.openpnp.spi.PropertySheetHolder;
import org.simpleframework.xml.Attribute;

public class CarouselFeeder extends ReferenceFeeder {
    @Attribute
    private int address = 0;
    
    @Attribute
    private int tray = 0;
    
    @Attribute
    private double partArea; 
    
    @Override
    public boolean canFeedToNozzle(Nozzle nozzle) {
        return true;
    }

    @Override
    public Location getPickLocation() throws Exception {
        return location;
    }

    @Override
    public void feed(Nozzle nozzle) throws Exception {
        FireStepDriver machineDriver = (FireStepDriver) ((ReferenceMachine) Configuration.get().getMachine()).getDriver();
        CarouselDriver carouselDriver = machineDriver.getCarouselDriver(address);
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

    public double getPartArea() {
        return partArea;
    }

    public void setPartArea(double partArea) {
        this.partArea = partArea;
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
