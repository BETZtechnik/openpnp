package org.firepick.delta;

import org.openpnp.machine.reference.camera.OpenCvCamera;
import org.openpnp.model.Location;
import org.simpleframework.xml.Attribute;

public class FpdCamera extends OpenCvCamera {

    @Attribute(required = true)
    double safeZ = 45;

    @Override
    public void moveToSafeZ(double speed) throws Exception {
        logger.debug("moveToSafeZ({})", new Object[] { speed } );
        Location l = new Location(getLocation().getUnits(), Double.NaN,
                Double.NaN, safeZ, Double.NaN);
        driver.moveTo(this, l, speed);
        machine.fireMachineHeadActivity(head);
    }

}
