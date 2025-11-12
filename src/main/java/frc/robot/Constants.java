// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class CanIDs {
    public final static int LEFT_FOLLOWER = 1;
    public final static int LEFT_LEADER = 2;
    public final static int RIGHT_FOLLOWER = 3;
    public final static int RIGHT_LEADER = 4;
    
    public final static int FLYWHEEL_LEFT = 5;
    public final static int FLYWHEEL_RIGHT = 6;
    public final static int ELEVATION_CONTROL = 7;
    public final static int ROTATION_CONTROL = 8;
    public final static int INTAKE_CONTROL = 9;
  
  }
  public static class PwmIDs {
    public final static int ROTATION_ENCODER = 1;
    public final static int ELEVATION_ENCODER = 2;
  }
  public static class Limits {
    public static class Turret {
      public final static double ROTATION_MIN = 0;
      public final static double ROTATION_MAX = 1;
      public final static double ELEVEATION_MIN = 0;
      public final static double ELEVATION_MAX = 1;
    }
  }
}
