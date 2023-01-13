// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  public static AHRS Gyro = new AHRS(Port.kMXP);
  public static final double MAX_ROBOT_SPEED = 0;
  public static PIDController FLPID = new PIDController(0.05, 0.1, 0.001);
  public static PIDController FRPID = new PIDController(0.05, 0.1, 0.001);
  public static PIDController BLPID = new PIDController(0.05, 0.1, 0.001);
  public static PIDController BRPID = new PIDController(0.05, 0.1, 0.001);
  public static TalonFX FLDriveMotor = new TalonFX(Constants.OperatorConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR);
  public static TalonFX FRDriveMotor = new TalonFX(Constants.OperatorConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR);
  public static TalonFX BLDriveMotor = new TalonFX(Constants.OperatorConstants.BACK_LEFT_MODULE_DRIVE_MOTOR);
  public static TalonFX BRDriveMotor = new TalonFX(Constants.OperatorConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR);
  public static TalonSRX FLSteerMotor = new TalonSRX(Constants.OperatorConstants.FRONT_LEFT_MODULE_STEER_MOTOR);
  public static TalonSRX FRSteerMotor = new TalonSRX(Constants.OperatorConstants.FRONT_RIGHT_MODULE_STEER_MOTOR);
  public static TalonSRX BLSteerMotor = new TalonSRX(Constants.OperatorConstants.BACK_LEFT_MODULE_STEER_MOTOR);
  public static TalonSRX BRSteerMotor = new TalonSRX(Constants.OperatorConstants.BACK_RIGHT_MODULE_STEER_MOTOR);
  public static Encoder FLEncoder = new Encoder(Constants.OperatorConstants.FRONT_LEFT_MODULE_STEER_ENCODER[0], Constants.OperatorConstants.FRONT_LEFT_MODULE_STEER_ENCODER[1]);
  public static Encoder FREncoder = new Encoder(Constants.OperatorConstants.FRONT_RIGHT_MODULE_STEER_ENCODER[0], Constants.OperatorConstants.FRONT_RIGHT_MODULE_STEER_ENCODER[1]);
  public static Encoder BLEncoder = new Encoder(Constants.OperatorConstants.BACK_LEFT_MODULE_STEER_ENCODER[0], Constants.OperatorConstants.BACK_LEFT_MODULE_STEER_ENCODER[1]);
  public static Encoder BREncoder = new Encoder(Constants.OperatorConstants.BACK_RIGHT_MODULE_STEER_ENCODER[0], Constants.OperatorConstants.BACK_RIGHT_MODULE_STEER_ENCODER[1]);
  Translation2d m_frontLeftLocation = new Translation2d(Units.inchesToMeters(6.5), Units.inchesToMeters(7));
  Translation2d m_frontRightLocation = new Translation2d(Units.inchesToMeters(6.5), Units.inchesToMeters(-7));
  Translation2d m_backLeftLocation = new Translation2d(Units.inchesToMeters(-6.5), Units.inchesToMeters(7));
  Translation2d m_backRightLocation = new Translation2d(Units.inchesToMeters(-6.5), Units.inchesToMeters(-7));
  
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  );

  // Example chassis speeds: 1 meter per second forward, 3 meters
// per second to the left, and rotation at 1.5 radians per second
// counterclockwise.
ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

// Convert to module states
SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

// Front left module state
SwerveModuleState frontLeft = moduleStates[0];

// Front right module state
SwerveModuleState frontRight = moduleStates[1];

// Back left module state
SwerveModuleState backLeft = moduleStates[2];

// Back right module state
SwerveModuleState backRight = moduleStates[3];



  /** Creates a new Drivetrain. */
  public Drivetrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    moduleStates = m_kinematics.toSwerveModuleStates(speeds);
    frontLeft = moduleStates[0];
    frontRight = moduleStates[1];
    backLeft = moduleStates[2];
    backRight = moduleStates[3];
    var frontLeftOptimized = SwerveModuleState.optimize(frontLeft, getEncoderValue(FLEncoder));
    var frontRightOptimized = SwerveModuleState.optimize(frontRight, getEncoderValue(FREncoder));
    var backLeftOptimized = SwerveModuleState.optimize(backLeft, getEncoderValue(BLEncoder));
    var backRIghtOptimized = SwerveModuleState.optimize(backRight, getEncoderValue(BREncoder));

  }

  public Rotation2d getEncoderValue(Encoder encoder) {
    return Rotation2d.fromDegrees((encoder.getDistance() * 1.12)/360);
  }

  public void driveWithMisery(double xSpeed, double ySpeed, double rad) {
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rad, getGyroYaw());
  }

  public Rotation2d getGyroYaw() {
    return Gyro.getRotation2d();
  }

  public void setState(SwerveModuleState state, TalonFX driveMotor, TalonSRX steerMotor, PIDController PID, Encoder encoder) {
    steerMotor.set(ControlMode.PercentOutput, PID.calculate(getEncoderValue(encoder).getRadians(), state.angle.getRadians()));
    driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond/MAX_ROBOT_SPEED);
  }
  
}
