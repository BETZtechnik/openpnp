package org.openpnp.model;

import org.simpleframework.xml.Attribute;

public class ReflowProfileStep extends AbstractModelObject {
    @Attribute
    protected String name = "Unknown";
    
    @Attribute
    protected double maxRateCelsiusPerSecond = 0d;
    
    @Attribute
    protected double targetTemperatureCelsius = 0d;
    
    @Attribute
    protected int holdTimeSeconds = 0;
    
    public ReflowProfileStep() {
        
    }

    public ReflowProfileStep(String name, double maxRateCelsiusPerSecond, double targetTemperatureCelsius, int holdTimeSeconds) {
        this.name = name;
        this.maxRateCelsiusPerSecond = maxRateCelsiusPerSecond;
        this.targetTemperatureCelsius = targetTemperatureCelsius;
        this.holdTimeSeconds = holdTimeSeconds;
    }
    
    public String getName() {
        return name;
    }

    public void setName(String name) {
        String oldValue = this.name;
        this.name = name;
        firePropertyChange("name", oldValue, name);
    }
    
    public double getMaxRateCelsiusPerSecond() {
        return maxRateCelsiusPerSecond;
    }
    
    public void setMaxRateCelsiusPerSecond(double maxRateCelsiusPerSecond) {
        double oldValue = this.maxRateCelsiusPerSecond;
        this.maxRateCelsiusPerSecond = maxRateCelsiusPerSecond;
        firePropertyChange("maxRateCelsiusPerSecond", oldValue, maxRateCelsiusPerSecond);
    }
    
    public double getTargetTemperatureCelsius() {
        return targetTemperatureCelsius;
    }
    
    public void setTargetTemperatureCelsius(double targetTemperatureCelsius) {
        double oldValue = this.targetTemperatureCelsius;
        this.targetTemperatureCelsius = targetTemperatureCelsius;
        firePropertyChange("targetTemperatureCelsius", oldValue, targetTemperatureCelsius);
    }
    
    public int getHoldTimeSeconds() {
        return holdTimeSeconds;
    }
    
    public void setHoldTimeSeconds(int holdTimeSeconds) {
        double oldValue = this.holdTimeSeconds;
        this.holdTimeSeconds = holdTimeSeconds;
        firePropertyChange("holdTimeSeconds", oldValue, holdTimeSeconds);
    }
}
