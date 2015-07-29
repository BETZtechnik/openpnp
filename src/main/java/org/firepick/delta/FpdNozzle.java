package org.firepick.delta;

import org.openpnp.machine.reference.ReferenceNozzle;
import org.openpnp.model.Location;
import org.simpleframework.xml.Attribute;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class FpdNozzle extends ReferenceNozzle   {
    private final static Logger logger = LoggerFactory
            .getLogger(FpdNozzle.class);

    @Attribute(required = true)
    double safeZ = 45;
    
	@Override
    public void moveToSafeZ(double speed) throws Exception {
		logger.debug("{}.moveToSafeZ({})", new Object[]{getName(), speed});
        Location l = new Location(getLocation().getUnits(), Double.NaN,
                Double.NaN, safeZ, Double.NaN);
        driver.moveTo(this, l, speed);
        machine.fireMachineHeadActivity(head);
    }

}
