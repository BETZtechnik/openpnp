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

package org.openpnp.machine.reference;

import org.openpnp.gui.support.Wizard;
import org.openpnp.model.BoardLocation;
import org.openpnp.model.Configuration;
import org.openpnp.model.ReflowProfileStep;
import org.openpnp.spi.Head;
import org.openpnp.spi.Machine;
import org.openpnp.spi.base.AbstractJobProcessor;
import org.simpleframework.xml.Attribute;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ReferenceReflowJobProcessor extends AbstractJobProcessor {
	private static final Logger logger = LoggerFactory.getLogger(ReferenceReflowJobProcessor.class);
	
	@Attribute(required=false)
	private String dummy;
	
	double currentTemp = 21;
	
	public ReferenceReflowJobProcessor() {
	}
	
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
		
		for (BoardLocation boardLocation : job.getBoardLocations()) {
		    for (ReflowProfileStep step : boardLocation.getBoard().getReflowProfile()) {
	            if (!shouldJobProcessingContinue()) {
	                return;
	            }
		        
		        try {
	                simulateStep(step);
		        }
		        catch (Exception e) {
		            fireJobEncounteredError(JobError.MachineMovementError, "Reflow step failed");
		        }
		    }
		}
		
		fireDetailedStatusUpdated("Job complete.");
		
		state = JobState.Stopped;
		fireJobStateChanged();
	}
	
	private void simulateStep(ReflowProfileStep step) throws Exception {
        double maxRate = 5;
        int tickMs = 250;

        fireDetailedStatusUpdated(step.getName());
	    double rate = step.getMaxRateCelsiusPerSecond();
	    if (rate == 0 || rate > maxRate) {
	        rate = maxRate;
	    }
	    rate = rate / (1000 / tickMs);
	    if (step.getTargetTemperatureCelsius() < currentTemp) {
	        // cooling
	        while (currentTemp > step.getTargetTemperatureCelsius() && shouldJobProcessingContinue()) {
	            currentTemp -= rate;
                fireReflowProgress(step, currentTemp, 0);
	            Thread.sleep(250);
	        }
	    }
	    else {
	        // heating
            while (currentTemp < step.getTargetTemperatureCelsius() && shouldJobProcessingContinue()) {
                currentTemp += rate;
                fireReflowProgress(step, currentTemp, 0);
                Thread.sleep(250);
            }
	    }
	    // hold
	    int holdTime = 0;
	    while (holdTime < step.getHoldTimeSeconds() && shouldJobProcessingContinue()) {
	        Thread.sleep(1000);
	        holdTime++;
	        fireReflowProgress(step, currentTemp, holdTime);
	    }
    }
	
    @Override
    public Wizard getConfigurationWizard() {
        return null;
    }
}
