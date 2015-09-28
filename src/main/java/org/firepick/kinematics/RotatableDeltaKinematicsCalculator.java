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

package org.firepick.kinematics;

import java.util.ArrayList;
import java.util.List;

import org.firepick.model.AngleTriplet;
import org.firepick.model.RawStepTriplet;
import org.openpnp.ConfigurationListener;
import org.openpnp.model.Configuration;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.simpleframework.xml.Attribute;
import org.simpleframework.xml.ElementList;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * So, we have to handle two situations. One for mapped axes (with encoders) and
 * one with pulley diameters.
 * 
 * This is primarily about converting angles to steps, and vice-versa. But
 * because we use home steps in one case and home angles in the other we also
 * have to manage some of the other attributes such as cartesian home location.
 * 
 * One of the key parameters we need to know is how to get the pulley arms
 * horizontal from their location when homed against the home switches. We call
 * this home angle. The home angle is, for ca lculation purposes, 0. Negative
 * angles move the arm towards the home switch and positive angles move it away
 * from the home switch.
 * 
 * Due to manufacturing differences, home angle needs to be determined
 * empirically. We do this by moving each arm individually to horizontal using a
 * jig and then commanding the motion controller to move the arm back to home
 * while counting steps. The resulting step count (minus any backoff steps) is
 * the number of steps required to move from home to horizontal.
 *
 * The original case of calculating steps is based on pulley diameter. To
 * calculate steps from angle the equation is: steps = angle *
 * (steps_per_rotation_of_stepper * microsteps * pulley_reduction / 360)
 * 
 * For the mapped axes we measure angles using an optical encoder and count
 * steps for some number of samples. Then we interpolate between angles for the
 * steps. This lets us cancel global error.
 */
public class RotatableDeltaKinematicsCalculator {
    private static final Logger logger = LoggerFactory.getLogger(RotatableDeltaKinematicsCalculator.class);

    enum Axis {
        X, Y, Z
    }
    
    /**
     * End effector length.
     */
    @Attribute(required = false)
    private double e = 131.636;

    /**
     * Base length.
     */
    @Attribute(required = false)
    private double f = 190.526;

    /**
     * Carbon rod length.
     */
    @Attribute(required = false)
    private double rE = 270.000;

    /**
     * Servo horn length.
     */
    @Attribute(required = false)
    private double rF = 90.000;

    // TODO: What is this?
    @Attribute(required = false)
    private double deltaZoffset = 268.000;

    // TODO: What is this?
    @Attribute(required = false)
    private double deltaEeOffset = 15.000;

    // TODO: What is this?
    @Attribute(required = false)
    private double toolOffset = 30.500;

    /**
     * Number of steps required to move the X motor off the home switch to where
     * the servo horn is horizontal with respect to the top plate.
     */
    @Attribute(required = false)
    private int homeAngleStepsX = -5440;

    /**
     * Number of steps required to move the Y motor off the home switch to where
     * the servo horn is horizontal with respect to the top plate.
     */
    @Attribute(required = false)
    private int homeAngleStepsY = -5439;

    /**
     * Number of steps required to move the Z motor off the home switch to where
     * the servo horn is horizontal with respect to the top plate.
     */
    @Attribute(required = false)
    private int homeAngleStepsZ = -5503;

    @Attribute(required = false)
    private double stepsPerMotorRotation = 200.0;

    @Attribute(required = false)
    private double motorMicrosteps = 16.0;

    @Attribute(required = false)
    private double pulleyReductionX = 9.463324361;

    @Attribute(required = false)
    private double pulleyReductionY = 9.488866397;

    @Attribute(required = false)
    private double pulleyReductionZ = 9.428427757;

    @ElementList(required = false)
    private List<StepsToAngleMapping> stepsToAngleMappings = new ArrayList<StepsToAngleMapping>();

    private double stepsPerDegreeX = (stepsPerMotorRotation * motorMicrosteps * pulleyReductionX) / 360.0;

    private double stepsPerDegreeY = (stepsPerMotorRotation * motorMicrosteps * pulleyReductionY) / 360.0;

    private double stepsPerDegreeZ = (stepsPerMotorRotation * motorMicrosteps * pulleyReductionZ) / 360.0;

    // TODO: What is this used for?
    private double zCalcOffset = ((deltaZoffset - toolOffset - deltaEeOffset) * -1);

    // Delta calc stuff
    private static final double SIN_120 = Math.sqrt(3.0) / 2.0;
    private static final double COS_120 = -0.5;
    private static final double TAN_60 = Math.sqrt(3.0);
    private static final double SIN_30 = 0.5;
    private static final double TAN_30 = 1 / Math.sqrt(3.0);
    
    public RotatableDeltaKinematicsCalculator() {
    }

    /**
     * Perform forward kinematics to convert steps to a cartesian Location.
     * @param steps
     * @return
     * @throws Exception
     */
    public Location getLocation(RawStepTriplet steps) throws Exception {
        return calculateForwardKinematics(getAngles(steps));
    }

    /**
     * Perform inverse kinematics to convert a cartesian location to steps.
     * @param location
     * @return
     * @throws Exception
     */
    public RawStepTriplet getRawSteps(Location location) throws Exception {
        return getRawSteps(calculateInverseKinematics(location));
    }

    public RawStepTriplet getHomeRawSteps() {
        return new RawStepTriplet(homeAngleStepsX, homeAngleStepsY, homeAngleStepsZ);
    }

    /**
     * Convert angles to raw steps. If interpolation mappings are available
     * they are used, otherwise the steps default to the steps calculated using
     * the base pulley diameters.
     * @param deltaCalc
     * @return
     */
    private RawStepTriplet getRawSteps(AngleTriplet angles) {
        RawStepTriplet basic = getRawStepsBasic(angles);
        RawStepTriplet interpolated = getRawStepsInterpolated(angles);
        RawStepTriplet ret = interpolated != null ? interpolated : basic; 
        logger.debug("getRawSteps({})", angles);
        logger.debug("basic        {}", basic);
        logger.debug("interpolated {}", interpolated);
        logger.debug("final        {}", ret);
        return ret;
    }
    
    private AngleTriplet getAngles(RawStepTriplet steps) {
        AngleTriplet basic = getAnglesBasic(steps);
        AngleTriplet interpolated = getAnglesInterpolated(steps);
        AngleTriplet ret = interpolated != null ? interpolated : basic; 
        logger.debug("getAngles({})", steps);
        logger.debug("basic        {}", basic);
        logger.debug("interpolated {}", interpolated);
        logger.debug("final        {}", ret);
        return ret;
    }

    // Get the raw steps for a specified angle
    private RawStepTriplet getRawStepsBasic(AngleTriplet angles) {
        int stepsX = (int) (angles.x * stepsPerDegreeX + 0.5d);
        int stepsY = (int) (angles.y * stepsPerDegreeY + 0.5d);
        int stepsZ = (int) (angles.z * stepsPerDegreeZ + 0.5d);
        return new RawStepTriplet(stepsX, stepsY, stepsZ);
    }
    
    // Get the raw steps for a specified angle
    private RawStepTriplet getRawStepsInterpolated(AngleTriplet angles) {
        try {
            int stepsX = interpolateAngleToSteps(angles.x, Axis.X);
            int stepsY = interpolateAngleToSteps(angles.y, Axis.Y);
            int stepsZ = interpolateAngleToSteps(angles.z, Axis.Z);
            return new RawStepTriplet(stepsX, stepsY, stepsZ);
        }
        catch (Exception e) {
            return null;
        }
    }
    
    /**
     * Interpolates an angle to a number of steps based on an input mapping
     * taken by measuring real world angles to steps using optical encoders.
     * @param angle
     * @param axis
     * @return
     */
    private int interpolateAngleToSteps(double angle, Axis axis) throws Exception {
        for (int i = 1; i < stepsToAngleMappings.size(); i++) {
            StepsToAngleMapping min = stepsToAngleMappings.get(i - 1);
            StepsToAngleMapping max = stepsToAngleMappings.get(i);
            int minSteps = min.steps;
            int maxSteps = max.steps;
            double minAngle = min.getAngle(axis);
            double maxAngle = max.getAngle(axis);
            if (angle >= minAngle && angle <= maxAngle) {
                double stepsPerDegree = (minSteps - maxSteps) / (minAngle - maxAngle);
                double angleDelta = angle - minAngle;
                int steps = (int) (minSteps + (angleDelta * stepsPerDegree) + 0.5d);
                return steps;
            }
        }
        throw new Exception("interpolateAngleToSteps: No valid interpolations found.");
    }
    
    private AngleTriplet getAnglesBasic(RawStepTriplet steps) {
        // TODO: handle interpolation
        return new AngleTriplet(getAngleFromRawSteps(stepsPerDegreeX, steps.x),
                getAngleFromRawSteps(stepsPerDegreeY, steps.y),
                getAngleFromRawSteps(stepsPerDegreeZ, steps.z));
    }
    
    private double getAngleFromRawSteps(double xyzSteps, int steps) {
        return (steps - 0.5d) / xyzSteps;
    }
    
    private AngleTriplet getAnglesInterpolated(RawStepTriplet steps) {
        try {
            return new AngleTriplet(
                    interpolateStepsToAngle(steps.x, Axis.X),
                    interpolateStepsToAngle(steps.y, Axis.Y),
                    interpolateStepsToAngle(steps.z, Axis.Z));
        }
        catch (Exception e) {
            return null;
        }
    }
    
    private double interpolateStepsToAngle(int steps, Axis axis) throws Exception {
        for (int i = 1; i < stepsToAngleMappings.size(); i++) {
            StepsToAngleMapping min = stepsToAngleMappings.get(i - 1);
            StepsToAngleMapping max = stepsToAngleMappings.get(i);
            int minSteps = min.steps;
            int maxSteps = max.steps;
            double minAngle = min.getAngle(axis);
            double maxAngle = max.getAngle(axis);
            if (steps >= minSteps && steps <= maxSteps) {
                double degreesPerStep = (minAngle - maxAngle) / (minSteps - maxSteps);
                int stepsDelta = steps - minSteps;
                // TODO: Don't forget the -0.5d somewhere.
                double angle = (minAngle + (stepsDelta * degreesPerStep));
                return angle;
            }
        }
        throw new Exception("interpolateStepsToAngle: No valid interpolations found.");
    }
    
    private AngleTriplet calculateInverseKinematics(Location cartesianLoc)
            throws RotatableDeltaKinematicsException {
        // trossen tutorial puts the "X" in the front/middle. FPD puts this arm
        // in the back/middle for aesthetics.
        double rotated_x = -1 * cartesianLoc.getX();
        double rotated_y = -1 * cartesianLoc.getY();
        double z_with_offset = cartesianLoc.getZ() + zCalcOffset; // The delta
                                                                  // calc below
                                                                  // places zero
                                                                  // at the top.
                                                                  // Subtract
                                                                  // the Z
                                                                  // offset to
                                                                  // make zero
                                                                  // at the
                                                                  // bottom.

        AngleTriplet solution = new AngleTriplet(0, 0, 0);
        try {
            solution.x = calculateInverseKinematicsYz(rotated_x, rotated_y,
                    z_with_offset);
            solution.y = calculateInverseKinematicsYz(rotated_x * COS_120
                    + rotated_y * SIN_120, rotated_y * COS_120 - rotated_x
                    * SIN_120, z_with_offset); // rotate coords to +120 deg
            solution.z = calculateInverseKinematicsYz(rotated_x * COS_120
                    - rotated_y * SIN_120, rotated_y * COS_120 + rotated_x
                    * SIN_120, z_with_offset); // rotate coords to -120 deg
            solution.successfulCalc = true;
        }
        catch (RotatableDeltaKinematicsException e) {
            throw new RotatableDeltaKinematicsException(
                    String.format(
                            "Delta calcInverse: Non-existing point for Cartesian location x=%.3f, y=%.3f, z=%.3f, Z_CALC_OFFSET=%.3f",
                            cartesianLoc.getX(), cartesianLoc.getY(),
                            cartesianLoc.getZ(), zCalcOffset));
        }
        return solution;
    }

    // Helper function for calculateDelta()
    private double calculateInverseKinematicsYz(double x, double y, double z)
            throws RotatableDeltaKinematicsException {

        double y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
        double y0 = y - (0.5 * 0.57735 * e); // shift center to edge
        // z = a + b*y
        double a = (x * x + y0 * y0 + z * z + rF * rF - rE * rE - y1 * y1)
                / (2 * z);
        double b = (y1 - y0) / z;
        // discriminant
        double d = -(a + b * y1) * (a + b * y1) + rF * (b * b * rF + rF);
        if (d < 0) {
            throw new RotatableDeltaKinematicsException(
                    "Delta calcInverse: Non-existing point"); // non-existing
                                                              // point
        }
        else {
            double yj = (y1 - a * b - Math.sqrt(d)) / (b * b + 1); // choosing
                                                                   // outer
                                                                   // point
            double zj = a + b * yj;
            return (180.0 * Math.atan(-zj / (y1 - yj)) / Math.PI + ((yj > y1) ? 180.0
                    : 0.0));
        }
        // return 0;

    }

    // forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
    // returned status: 0=OK, -1=non-existing position
    private Location calculateForwardKinematics(AngleTriplet angles)
            throws RotatableDeltaKinematicsException {
        double t = (f - e) * TAN_30 / 2;
        double dtr = Math.PI / 180.0;
        double theta1 = angles.x * dtr;
        double theta2 = angles.y * dtr;
        double theta3 = angles.z * dtr;

        double y1 = -(t + rF * Math.cos(theta1));
        double z1 = -rF * Math.sin(theta1);

        double y2 = (t + rF * Math.cos(theta2)) * SIN_30;
        double x2 = y2 * TAN_60;
        double z2 = -rF * Math.sin(theta2);

        double y3 = (t + rF * Math.cos(theta3)) * SIN_30;
        double x3 = -y3 * TAN_60;
        double z3 = -rF * Math.sin(theta3);

        double dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

        double w1 = y1 * y1 + z1 * z1;
        double w2 = x2 * x2 + y2 * y2 + z2 * z2;
        double w3 = x3 * x3 + y3 * y3 + z3 * z3;

        // x = (a1*z + b1)/dnm
        double a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
        double b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

        // y = (a2*z + b2)/dnm;
        double a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
        double b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

        // a*z^2 + b*z + c = 0
        double a = a1 * a1 + a2 * a2 + dnm * dnm;
        double b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
        double c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm
                * (z1 * z1 - rE * rE);

        // discriminant
        double d = b * b - (float) 4.0 * a * c;
        if (d < 0) {
            throw new RotatableDeltaKinematicsException(
                    String.format(
                            "Delta calcForward: Non-existing point for angles x=%.3f, y=%.3f, z=%.3f",
                            angles.x, angles.y, angles.z));
        }

        double z = -(double) 0.5 * (b + Math.sqrt(d)) / a;
        double x = (a1 * z + b1) / dnm;
        double y = (a2 * z + b2) / dnm;

        z -= zCalcOffset; // NJ

        return new Location(LengthUnit.Millimeters, x, y, z, 0);
    }
    
    @SuppressWarnings("serial")
    public static class RotatableDeltaKinematicsException extends Exception {
        public RotatableDeltaKinematicsException(String message) {
            super(message);
        }
    }

    public static class StepsToAngleMapping {
        @Attribute
        private int steps;

        @Attribute
        private double angleX;

        @Attribute
        private double angleY;

        @Attribute
        private double angleZ;
        
        public StepsToAngleMapping() {
        }
        
        public StepsToAngleMapping(int steps, double angleX, double angleY, double angleZ) {
            this.steps = steps;
            this.angleX = angleX;
            this.angleY = angleY;
            this.angleZ = angleZ;
        }
        
        public double getAngle(Axis axis) {
            switch (axis) {
                case X:
                    return angleX;
                case Y:
                    return angleY;
                case Z:
                    return angleZ;
                default:
                    return Double.NaN;
            }
        }

        public int getSteps() {
            return steps;
        }

        public void setSteps(int steps) {
            this.steps = steps;
        }

        public double getAngleX() {
            return angleX;
        }

        public void setAngleX(double angleX) {
            this.angleX = angleX;
        }

        public double getAngleY() {
            return angleY;
        }

        public void setAngleY(double angleY) {
            this.angleY = angleY;
        }

        public double getAngleZ() {
            return angleZ;
        }

        public void setAngleZ(double angleZ) {
            this.angleZ = angleZ;
        }

        @Override
        public String toString() {
            return "StepsToAngleMapping [steps=" + steps + ", angleX=" + angleX
                    + ", angleY=" + angleY + ", angleZ=" + angleZ + "]";
        }
    }
}
