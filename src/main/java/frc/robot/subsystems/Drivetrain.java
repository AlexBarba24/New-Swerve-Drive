// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  public static AHRS Gyro = new AHRS(Port.kMXP);
  public static final double MAX_ROBOT_SPEED = Constants.OperatorConstants.maxSpeed;
  public static PIDController FLPID = new PIDController(Constants.OperatorConstants.FLPID[0], Constants.OperatorConstants.FLPID[1], Constants.OperatorConstants.FLPID[2]);
  public static PIDController FRPID = new PIDController(Constants.OperatorConstants.FRPID[0], Constants.OperatorConstants.FRPID[1], Constants.OperatorConstants.FRPID[2]);
  public static PIDController BLPID = new PIDController(Constants.OperatorConstants.BLPID[0], Constants.OperatorConstants.BLPID[1], Constants.OperatorConstants.BLPID[2]);
  public static PIDController BRPID = new PIDController(Constants.OperatorConstants.BRPID[0], Constants.OperatorConstants.BRPID[1], Constants.OperatorConstants.BRPID[2]);
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
  
  public boolean isZeroed = true;

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
  public Drivetrain() {
    // FRDriveMotor.setInverted(true);
    // BRDriveMotor.setInverted(true);
    FLDriveMotor.setNeutralMode(NeutralMode.Brake);
    FRDriveMotor.setNeutralMode(NeutralMode.Brake);
    BLDriveMotor.setNeutralMode(NeutralMode.Brake);
    BRDriveMotor.setNeutralMode(NeutralMode.Brake);
    FLDriveMotor.setSelectedSensorPosition(0);
    FRDriveMotor.setSelectedSensorPosition(0);
    BLDriveMotor.setSelectedSensorPosition(0);
    BRDriveMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(isZeroed){
      frontLeft = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
      frontRight = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
      backLeft = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
      backRight = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    }else{
      moduleStates = m_kinematics.toSwerveModuleStates(speeds);
      frontLeft = moduleStates[0];
      frontRight = moduleStates[1];
      backLeft = moduleStates[2];
      backRight = moduleStates[3];
      frontLeft = SwerveModuleState.optimize(frontLeft, getEncoderValue(FLEncoder));
      frontRight = SwerveModuleState.optimize(frontRight, getEncoderValue(FREncoder));
      backLeft = SwerveModuleState.optimize(backLeft, getEncoderValue(BLEncoder));
      backRight = SwerveModuleState.optimize(backRight, getEncoderValue(BREncoder));
    }
    updateSmartDashboard();
    setState(frontLeft, FLDriveMotor, FLSteerMotor, FLPID, FLEncoder, Constants.OperatorConstants.FLSPeedScale);
    setState(frontRight, FRDriveMotor, FRSteerMotor, FRPID, FREncoder, Constants.OperatorConstants.FRSPeedScale);
    setState(backLeft, BLDriveMotor, BLSteerMotor, BLPID, BLEncoder, Constants.OperatorConstants.BLSPeedScale);
    setState(backRight, BRDriveMotor, BRSteerMotor, BRPID, BREncoder, Constants.OperatorConstants.BRSPeedScale);
  }

  public Rotation2d getEncoderValue(Encoder encoder) {
    return Rotation2d.fromDegrees((encoder.get() * (1.0/1.2)));
  }

  public void driveWithMisery(double xSpeed, double ySpeed, double rad) {
    isZeroed = false;
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rad, getGyroYaw());
  }

  public Rotation2d getGyroYaw() {
    return Gyro.getRotation2d();
  }

  public void setState(SwerveModuleState state, TalonFX driveMotor, TalonSRX steerMotor, PIDController PID, Encoder encoder, double speedScale) {
    steerMotor.set(ControlMode.PercentOutput, PID.calculate(getEncoderValue(encoder).getDegrees(), roundAngle(state.angle.getDegrees())));
    driveMotor.set(ControlMode.PercentOutput, (state.speedMetersPerSecond/MAX_ROBOT_SPEED)*speedScale);
  }
  
  public void updateSmartDashboard(){
    SmartDashboard.putNumber("FLSetSpeed", frontLeft.speedMetersPerSecond);
    SmartDashboard.putNumber("FLSetAngle", frontLeft.angle.getDegrees());
    SmartDashboard.putNumber("FRSetSpeed", frontRight.speedMetersPerSecond);
    SmartDashboard.putNumber("FRSetAngle", frontRight.angle.getDegrees());    
    SmartDashboard.putNumber("BLSetSpeed", backLeft.speedMetersPerSecond);
    SmartDashboard.putNumber("BLSetAngle", backLeft.angle.getDegrees());
    SmartDashboard.putNumber("BRSetSpeed", backRight.speedMetersPerSecond);
    SmartDashboard.putNumber("BRSetAngle", backRight.angle.getDegrees());
    SmartDashboard.putNumber("FLAngle", getEncoderValue(FLEncoder).getDegrees());
    SmartDashboard.putNumber("FRAngle", getEncoderValue(FREncoder).getDegrees());
    SmartDashboard.putNumber("BLAngle", getEncoderValue(BLEncoder).getDegrees());
    SmartDashboard.putNumber("BRAngle", getEncoderValue(BREncoder).getDegrees());
    SmartDashboard.putNumber("DRIVE ENCODER: FL", FLDriveMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("DRIVE ENCODER: FR", FRDriveMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("DRIVE ENCODER: BL", BLDriveMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("DRIVE ENCODER: BR", BRDriveMotor.getSelectedSensorPosition());
    // SmartDashboard.putNumber("RAW ENCODER: FL", FLEncoder.get());
    // SmartDashboard.putNumber("RAW ENCODER: FR", FREncoder.get());
    // SmartDashboard.putNumber("RAW ENCODER: BL", BLEncoder.get());
    // SmartDashboard.putNumber("RAW ENCODER: BR", BREncoder.get());
  }

  public void zeroMotors(){
    frontLeft = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    frontRight = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    backLeft = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    backRight = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    isZeroed = true;
  }

  public double roundAngle(double angle){
    return Math.round(angle * 1.2)/1.2;
  }

  public void PIDTuner(){
    FLSteerMotor.set(ControlMode.PercentOutput, FLPID.calculate(getEncoderValue(FLEncoder).getDegrees(), 90));
    FRSteerMotor.set(ControlMode.PercentOutput, FRPID.calculate(getEncoderValue(FREncoder).getDegrees(), 90));
    BLSteerMotor.set(ControlMode.PercentOutput, BLPID.calculate(getEncoderValue(BLEncoder).getDegrees(), 90));
    BRSteerMotor.set(ControlMode.PercentOutput, BRPID.calculate(getEncoderValue(BREncoder).getDegrees(), 90));
  }

  public void SpeedTuner(){
    FLDriveMotor.set(ControlMode.PercentOutput, .5*Constants.OperatorConstants.FLSPeedScale);
    FRDriveMotor.set(ControlMode.PercentOutput, .5*Constants.OperatorConstants.FRSPeedScale);
    BLDriveMotor.set(ControlMode.PercentOutput, .5*Constants.OperatorConstants.BLSPeedScale);
    BRDriveMotor.set(ControlMode.PercentOutput, .5*Constants.OperatorConstants.BRSPeedScale);
  }

  public void resetGyro(){
    Gyro.reset();
  }

}
