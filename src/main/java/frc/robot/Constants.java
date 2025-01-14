// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int PNEUMATIC_HUB_ID = 13;
    public static final int COMPRESSOR_ID = 13;
    public static final int SOLENOID_ID_IN = 0;
    public static final int SOLENOID_ID_OUT = 1;
    public static final int WINCH_MOTOR_ID = 6;
    public static final int PIVOT_MOTOR_ID = 0;
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8; 
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; 
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1; 
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 9; 
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; 
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5; 
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3; 
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7; 
        public static final int[] FRONT_LEFT_MODULE_STEER_ENCODER = {4,5}; 
        public static final int[] FRONT_RIGHT_MODULE_STEER_ENCODER = {0,1}; 
        public static final int[] BACK_LEFT_MODULE_STEER_ENCODER = {6,7}; 
        public static final int[] BACK_RIGHT_MODULE_STEER_ENCODER = {2,3}; 
        public static final int[] PIVOT_MOTOR_ENCODER = {9, 8};
        public static final double driveSpeedScale = 0.45; //Speed scale from 0 to 1
        public static final double maxSpeed = 8.95;
        public static final double rotationSpeedScale = maxSpeed/Math.hypot(Units.inchesToMeters(14.5)/2, Units.inchesToMeters(15.5)/2);
        public static final double[] FRPID = {0.03, 0, 0.0};
        public static final double[] FLPID = {0.03, 0, 0.0};
        public static final double[] BRPID = {0.03, 0, 0.0};
        public static final double[] BLPID = {0.03, 0, 0.0};
        public static final double[] AimPID = {0.215, 0, 0.0};
        public static final double FLSPeedScale = 1;
        public static final double FRSPeedScale = 1;
        public static final double BLSPeedScale = 1;
        public static final double BRSPeedScale = 1;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final double driveWheelRatio = 1/6.67;
        public static final double kPXController = 3;
        public static final double kPThetaController = 3;
        public static final int spikeRelayID = 1;
        public static final double[] armPID = {0.1, 0.02, 0.001};
        public static final double[] winchPID = {5, 0, 0};
        //public static final Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(maxSpeed/Math.hypot(Units.inchesToMeters(14.5)/2, Units.inchesToMeters(15.5)/2)/((Units.inchesToMeters(24)*Math.PI)/2), maxSpeed/Math.hypot(Units.inchesToMeters(14.5)/2, Units.inchesToMeters(15.5)/2)/((Units.inchesToMeters(24)*Math.PI)/2));;
        public static final double kMaxSpeerMetersPerSecond = maxSpeed; 
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadPerSecond = (2*2*Math.PI);
        public static final double kMaxAngularAccelerationRadPerSecondSquared = 3;
        public static final HashMap<String, Command> eventMap = new HashMap<>(); 
        public static final Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(kMaxAngularSpeedRadPerSecond, kMaxAngularAccelerationRadPerSecondSquared);

  }
}
