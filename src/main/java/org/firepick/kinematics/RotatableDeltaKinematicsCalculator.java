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

import org.firepick.model.AngleTriplet;
import org.firepick.model.RawStepTriplet;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;

public class RotatableDeltaKinematicsCalculator {

    //TODO: These should eventually become configurable.
	private double DELTA_Z_OFFSET = 268.000;
	private double DELTA_EE_OFFS = 15.000;
	private double TOOL_OFFSET = 30.500;
	private double Z_CALC_OFFSET = ((DELTA_Z_OFFSET - TOOL_OFFSET - DELTA_EE_OFFS) * -1);
	
	private double HOME_ANGLE_X  = -64.165; // Angle of the "X" endstop sensor (0=horizontal)
	private double HOME_ANGLE_Y  = -64.165; // Angle of the "Y" endstop sensor (0=horizontal)
	private double HOME_ANGLE_Z  = -64.165; // Angle of the "Z" endstop sensor (0=horizontal)

	private double deltaE  = 131.636; // End effector length
	private double deltaF  = 190.526; // Base length
	private double deltaRe = 268.000; // Carbon rod length
	private double deltaRf = 90.000;  // Servo horn length
	
	private double XYZ_FULL_STEPS_PER_ROTATION = 200.0;
	private double XYZ_MICROSTEPS = 16.0;
    private double PULLEY_REDUCTION_X = 9.55882352925; // validated for X 
    private double PULLEY_REDUCTION_Y = 9.55882352925; // validated for Y 
    private double PULLEY_REDUCTION_Z = 9.60979505961; // validated for Z
    private double X_STEPS = (XYZ_FULL_STEPS_PER_ROTATION*XYZ_MICROSTEPS*PULLEY_REDUCTION_X)/360.0;
    private double Y_STEPS = (XYZ_FULL_STEPS_PER_ROTATION*XYZ_MICROSTEPS*PULLEY_REDUCTION_Y)/360.0;
    private double Z_STEPS = (XYZ_FULL_STEPS_PER_ROTATION*XYZ_MICROSTEPS*PULLEY_REDUCTION_Z)/360.0;
	
	private int HOME_STEPS_X = -5440;
	private int HOME_STEPS_Y = -5439;
	private int HOME_STEPS_Z = -5503;
	
	public class RotatableDeltaKinematicsException extends Exception {
		public RotatableDeltaKinematicsException(String message){
		     super(message);
		}
	}
	
	public double getRe() {
	    return deltaRe;
	}
	
	public double getRf() {
	    return deltaRf;
	}
	
	public double getE() {
	    return deltaE;
	}
	
	public double getF() {
	    return deltaF;
	}
	
//	public double getGr() {
//	    return PULLEY_REDUCTION;
//	}
	
//    public double getHa1() {
//        return HOME_ANGLE_X;
//    }
//    
//    public double getHa2() {
//        return HOME_ANGLE_Y;
//    }
//    
//    public double getHa3() {
//        return HOME_ANGLE_Z;
//    }
//    
//    public double getMi() {
//        return XYZ_MICROSTEPS;
//    }
    
    public double getZo() {
        return Z_CALC_OFFSET;
    }
    
//    public double getSt() {
//        return XYZ_FULL_STEPS_PER_ROTATION;
//    }
    
	//Return raw steps, given an angle
	public int getRawStepsFromAngle(double xyzSteps, double angle)
	{
		return (int)(angle * xyzSteps + 0.5d);
	}
	
	public double getAngleFromRawSteps(double xyzSteps, int steps) {
	    return (steps - 0.5d) / xyzSteps;
	}
	
	public AngleTriplet getAnglesFromRawSteps(RawStepTriplet steps) {
	    return new AngleTriplet(
	            getAngleFromRawSteps(X_STEPS, steps.x), 
	            getAngleFromRawSteps(Y_STEPS, steps.y), 
	            getAngleFromRawSteps(Z_STEPS, steps.z));
	}
	
	public Location getLocation(RawStepTriplet steps) throws Exception {
	    return delta_calcForward(getAnglesFromRawSteps(steps));
	}
	
	public RawStepTriplet getRawSteps(Location location) throws Exception {
	    return getRawSteps(calculateDelta(location));
	}
	
	//Get the raw step home positions for the three axes
	public RawStepTriplet getHomePosRaw()
	{
	    // TODO: Hardcoded from backwards z probe test.
		return new RawStepTriplet(HOME_STEPS_X, HOME_STEPS_Y, HOME_STEPS_Z);
	}
	
	//Get the homing angles for the three axes
	public AngleTriplet getHomePos()
	{
		return new AngleTriplet(HOME_ANGLE_X, HOME_ANGLE_Y, HOME_ANGLE_Z);
	}
	
	public Location getHomePosCartesian() throws RotatableDeltaKinematicsException {
		return delta_calcForward(getHomePos());
	}
	
	public RawStepTriplet getRawSteps(AngleTriplet deltaCalc) {
	    return getRawStepsMapped(deltaCalc);
	}
	
    //Get the raw steps for a specified angle
    public RawStepTriplet getRawStepsPulleyDiameter(AngleTriplet deltaCalc)
    {
        return new RawStepTriplet(
                getRawStepsFromAngle(X_STEPS, deltaCalc.x),
                getRawStepsFromAngle(Y_STEPS, deltaCalc.y),
                getRawStepsFromAngle(Z_STEPS, deltaCalc.z));
    }
    
    //Get the raw steps for a specified angle
    public RawStepTriplet getRawStepsMapped(AngleTriplet deltaCalc)
    {
        double[][] table =  new double[][] {
                {-5550,-66.132,-65.628,-66.132},
                {-5000,-59.508,-59.076,-59.544},
                {-4500,-53.532,-53.172,-53.604},
                {-4000,-47.592,-47.268,-47.664},
                {-3500,-41.652,-41.364,-41.724},
                {-3000,-35.676,-35.496,-35.748},
                {-2500,-29.7,-29.556,-29.808},
                {-2000,-23.76,-23.652,-23.868},
                {-1500,-17.784,-17.748,-17.928},
                {-1000,-11.844,-11.844,-11.952},
                {-500,-5.904,-5.904,-5.976},
                {0,0,0.036,0},
                {500,5.976,5.976,5.976},
                {1000,11.916,11.916,11.952},
                {1500,17.856,17.856,17.928},
                {2000,23.796,23.796,23.904},
                {2500,29.7,29.772,29.88},
                {3000,35.64,35.712,35.892},
                {3500,41.544,41.652,41.868},
                {4000,47.484,47.628,47.844},
                {4500,53.388,53.604,53.82},
                {5000,59.328,59.544,59.796},
                {5500,65.232,65.52,65.772}
        };
        
        // find the index of the first angle that is greater than
        // the input
        // find the bounding angles in the table for the axis
        // determine the difference between the angles and the
        // steps for the bounds
        // create a ratio between the two
        // scale the bound start by the ratio to get the output
        RawStepTriplet baseline = getRawStepsPulleyDiameter(deltaCalc);
        int stepsX = baseline.x, stepsY = baseline.y, stepsZ = baseline.z;
        // Note we skip the first point because if we find the value at
        // the first point we don't have a lower bound to calculate from.
        boolean foundX = false, foundY = false, foundZ = false;
        for (int i = 1; i < table.length; i++) {
            if (!foundX && table[i - 1][1] <= deltaCalc.x && table[i][1] >= deltaCalc.x) {
                double stepsPerDegree = (table[i][0] - table[i - 1][0]) / (table[i][1] - table[i - 1][1]);
                int lowerBoundSteps = (int) table[i - 1][0];
                double angleDifference = deltaCalc.x - table[i - 1][1];
                stepsX = (int) (lowerBoundSteps + (angleDifference * stepsPerDegree));
                foundX = true;
            }
            if (!foundY && table[i - 1][2] <= deltaCalc.y && table[i][2] >= deltaCalc.y) {
                double stepsPerDegree = (table[i][0] - table[i - 1][0]) / (table[i][2] - table[i - 1][2]);
                int lowerBoundSteps = (int) table[i - 1][0];
                double angleDifference = deltaCalc.y - table[i - 1][2];
                stepsY = (int) (lowerBoundSteps + (angleDifference * stepsPerDegree));
                foundY = true;
            }
            if (!foundZ && table[i - 1][3] <= deltaCalc.z && table[i][3] >= deltaCalc.z) {
                double stepsPerDegree = (table[i][0] - table[i - 1][0]) / (table[i][3] - table[i - 1][3]);
                int lowerBoundSteps = (int) table[i - 1][0];
                double angleDifference = deltaCalc.z - table[i - 1][3];
                stepsZ = (int) (lowerBoundSteps + (angleDifference * stepsPerDegree));
                foundZ = true;
            }
        }
        
        RawStepTriplet mapped = new RawStepTriplet(stepsX, stepsY, stepsZ);
        System.out.println(deltaCalc);
        System.out.println(baseline);
        System.out.println(mapped);
        System.out.println();
        return mapped;
    }
    
	//Delta calc stuff
    private static final double sin120 = Math.sqrt(3.0) / 2.0;
    private static final double cos120 = -0.5;
    private static final double tan60 = Math.sqrt(3.0);
    private static final double sin30 = 0.5;
    private static final double tan30 = 1 / Math.sqrt(3.0);
    
    public AngleTriplet calculateDelta(Location cartesianLoc) throws RotatableDeltaKinematicsException 
    {
  	  //trossen tutorial puts the "X" in the front/middle. FPD puts this arm in the back/middle for aesthetics.
  	  double rotated_x = -1 * cartesianLoc.getX();
  	  double rotated_y = -1 * cartesianLoc.getY();
  	  double z_with_offset = cartesianLoc.getZ() + Z_CALC_OFFSET; //The delta calc below places zero at the top.  Subtract the Z offset to make zero at the bottom.
  	  
	  AngleTriplet solution = new AngleTriplet(0,0,0);
  	  try
  	  {
  	  	  solution.x = calculateYZ(rotated_x,                           rotated_y,                         z_with_offset);
  	  	  solution.y = calculateYZ(rotated_x*cos120 + rotated_y*sin120, rotated_y*cos120-rotated_x*sin120, z_with_offset);  // rotate coords to +120 deg
  	  	  solution.z = calculateYZ(rotated_x*cos120 - rotated_y*sin120, rotated_y*cos120+rotated_x*sin120, z_with_offset);  // rotate coords to -120 deg
  	  	  solution.successfulCalc = true;
  	  }
  	  catch (RotatableDeltaKinematicsException e)
  	  {
  		  throw new RotatableDeltaKinematicsException(String.format("Delta calcInverse: Non-existing point for Cartesian location x=%.3f, y=%.3f, z=%.3f, Z_CALC_OFFSET=%.3f", cartesianLoc.getX(), cartesianLoc.getY(), cartesianLoc.getZ(),Z_CALC_OFFSET ));
  	  }
  	  return solution;
    }
    
    //Helper function for calculateDelta()
	private double calculateYZ(double x, double y, double z) throws RotatableDeltaKinematicsException {

  	  double y1 = -0.5 * 0.57735 * deltaF; // f/2 * tg 30
      double y0 = y - (0.5 * 0.57735       * deltaE);    // shift center to edge
      // z = a + b*y
      double a = (x*x + y0*y0 + z*z +deltaRf*deltaRf - deltaRe*deltaRe - y1*y1) / (2*z);
      double b = (y1-y0)/z;
      // discriminant
      double d = -(a+b*y1)*(a+b*y1)+deltaRf*(b*b*deltaRf+deltaRf); 
      if (d < 0)
      {
    	  throw new RotatableDeltaKinematicsException("Delta calcInverse: Non-existing point"); // non-existing point
      }
      else
      {
          double yj = (y1 - a*b - Math.sqrt(d))/(b*b + 1); // choosing outer point
          double zj = a + b*yj;
          return (180.0*Math.atan(-zj/(y1 - yj))/Math.PI + ((yj>y1)?180.0:0.0));
      }
      //return 0;

	}
	
	// forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
	// returned status: 0=OK, -1=non-existing position
	public Location delta_calcForward(AngleTriplet angles)  throws RotatableDeltaKinematicsException 
	{
	    double t = (deltaF-deltaE)*tan30/2;
	    double dtr = Math.PI/180.0;
	    double theta1 = angles.x * dtr;
	    double theta2 = angles.y * dtr;
	    double theta3 = angles.z * dtr;
	 
	    double y1 = -(t + deltaRf * Math.cos(theta1));
	    double z1 = -deltaRf * Math.sin(theta1);
	 
	    double y2 = (t + deltaRf * Math.cos(theta2)) * sin30;
	    double x2 = y2 * tan60;
	    double z2 = -deltaRf * Math.sin(theta2);
	 
	    double y3 = (t + deltaRf * Math.cos(theta3)) * sin30;
	    double x3 = -y3 * tan60;
	    double z3 = -deltaRf * Math.sin(theta3);
	 
	    double dnm = (y2-y1)*x3-(y3-y1)*x2;
	 
	    double w1 = y1*y1 + z1*z1;
	    double w2 = x2*x2 + y2*y2 + z2*z2;
	    double w3 = x3*x3 + y3*y3 + z3*z3;
	     
	    // x = (a1*z + b1)/dnm
	    double a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
	    double b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
	 
	    // y = (a2*z + b2)/dnm;
	    double a2 = -(z2-z1)*x3+(z3-z1)*x2;
	    double b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
	 
	    // a*z^2 + b*z + c = 0
	    double a = a1*a1 + a2*a2 + dnm*dnm;
	    double b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
	    double c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - deltaRe*deltaRe);
	  
	    // discriminant
	    double d = b*b - (float)4.0*a*c;
	    if (d < 0)
    	{
	    	throw new RotatableDeltaKinematicsException(String.format("Delta calcForward: Non-existing point for angles x=%.3f, y=%.3f, z=%.3f", angles.x, angles.y, angles.z));
    	}
	 
	    
	    double z = -(double)0.5 * (b+Math.sqrt(d))/a;
	    double x = (a1*z + b1)/dnm;
	    double y = (a2*z + b2)/dnm;
	    
	    z -= Z_CALC_OFFSET; //NJ
	    
	    return new Location(LengthUnit.Millimeters, x, y, z, 0);
	}
}
