package org.firepick.delta;

import org.openpnp.machine.reference.ReferenceHead;

public class FpdHead extends ReferenceHead {
    @Override
    public void moveToSafeZ(double speed) throws Exception {
        // Since everything is on the same head, don't bother Safe-Z'ing
        // every object. Just do the first nozzle and call it a day.
        nozzles.get(0).moveToSafeZ(speed);
    }
}
