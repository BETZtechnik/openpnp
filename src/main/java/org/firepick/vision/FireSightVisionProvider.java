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

package org.firepick.vision;

import java.awt.image.BufferedImage;
import java.util.List;

import org.firepick.delta.FireSight;
import org.firepick.delta.FireSight.FireSightResult;
import org.opencv.core.RotatedRect;
import org.openpnp.machine.reference.vision.OpenCvVisionProvider;
import org.openpnp.model.Location;
import org.openpnp.model.Part;
import org.openpnp.spi.Camera.Looking;
import org.openpnp.spi.Nozzle;
import org.openpnp.util.VisionUtils;

public class FireSightVisionProvider extends OpenCvVisionProvider {
    @Override
    public Location getPartBottomOffsets(Part part, Nozzle nozzle)
            throws Exception {
        if (camera.getLooking() != Looking.Up) {
            throw new Exception("Bottom vision only implemented for Up looking cameras");
        }

        // Position the part above the center of the camera.
        // First move to Safe-Z.
        nozzle.moveToSafeZ(1.0);
        // Then move to the camera in X, Y at Safe-Z and rotate the
        // part to 0.
        nozzle.moveTo(camera.getLocation().derive(null, null, Double.NaN, 0.0), 1.0);
        // Then lower the part to the Camera's focal point in Z. Maintain the 
        // part's rotation at 0.
        nozzle.moveTo(camera.getLocation().derive(null, null, null, Double.NaN), 1.0);
        
        for (int i = 0; i < 3; i++) {
            // Settle
            Thread.sleep(750);
            // Grab an image.
            BufferedImage image = camera.capture();
            
            // perform the vision op
            FireSightResult result = FireSight.fireSight(image, "bottomVision.json");
            List<RotatedRect> rects = FireSight.parseRotatedRects(
                    result.model.get("minAreaRect").getAsJsonObject().get("rects").getAsJsonArray());
            RotatedRect rect = rects.get(0);
            
            // Create the offsets object
            Location offsets = VisionUtils.getPixelCenterOffsets(
                    camera,
                    rect.center.x, 
                    rect.center.y);
            offsets = offsets.derive(null, null, null, rect.angle);

            // Invert Y, since our camera is upside down. Figure out how to handle this in config.
            offsets = offsets.multiply(1, -1, 1, 1);

            // Move the nozzle so that the part is oriented correctly over the
            // camera.
            Location location = nozzle.getLocation();
            location = location.subtractWithRotation(offsets);
            
            nozzle.moveTo(location, 1.0);
        }
        
        // Chill a tick to show off our work.
        Thread.sleep(2500);
        
        Location offsets = camera.getLocation().subtractWithRotation(nozzle.getLocation());

        // Return to Safe-Z just to be safe.
        nozzle.moveToSafeZ(1.0);
        return offsets;
    } 
}
