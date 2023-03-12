// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Instrum;
/**Drivetrain subsystem.
 * 
 */
public class Drivetrain extends SubsystemBase {
  StringBuilder _sb = new StringBuilder();

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

  public  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  // Example chassis speeds: 1 meter per second forward, 3 meters
// per second to the left, and rotation at 1.5 radians per second
// counterclockwise.
ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

// Convert to module states
SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
SwerveModulePosition[] modulePositions = updateModulePos();
// Front left module state
SwerveModuleState frontLeft = moduleStates[0];

// Front right module state
SwerveModuleState frontRight = moduleStates[1];

// Back left module state
SwerveModuleState backLeft = moduleStates[2];

// Back right module state
SwerveModuleState backRight = moduleStates[3];

Field2d field = new Field2d();

SwerveDriveOdometry odometry = new SwerveDriveOdometry(m_kinematics, getGyroPitch(), modulePositions);

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    FLDriveMotor.setSelectedSensorPosition(0);
    FRDriveMotor.setSelectedSensorPosition(0);
    BLDriveMotor.setSelectedSensorPosition(0);
    BRDriveMotor.setSelectedSensorPosition(0);
    // FLDriveMotor.configOpenloopRamp(2);
    // FRDriveMotor.configOpenloopRamp(2);
    // BLDriveMotor.configOpenloopRamp(2);
    // BRDriveMotor.configOpenloopRamp(2);
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

  public void configureFalcon(TalonFX _talon){
    		/* Factory Default all hardware to prevent unexpected behaviour */
		_talon.configFactoryDefault();
		
		/* Config neutral deadband to be the smallest possible */
		_talon.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
        _talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
											

		/* Config the peak and nominal outputs */
		_talon.configNominalOutputForward(0, 30);
		_talon.configNominalOutputReverse(0, 30);
		_talon.configPeakOutputForward(.5, 30);
		_talon.configPeakOutputReverse(-.5, 30);

		/* Config the Velocity closed loop gains in slot0 */
		_talon.config_kF(0, 0.04721901685, 30);
		_talon.config_kP(0, 0, 30);
		_talon.config_kI(0, 0, 30);
		_talon.config_kD(0, 0, 30);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _talon.setSensorPhase(true);
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
    setState(frontLeft, FLDriveMotor, FLSteerMotor, FLPID, FLEncoder, Constants.OperatorConstants.FLSPeedScale);
    setState(frontRight, FRDriveMotor, FRSteerMotor, FRPID, FREncoder, Constants.OperatorConstants.FRSPeedScale);
    setState(backLeft, BLDriveMotor, BLSteerMotor, BLPID, BLEncoder, Constants.OperatorConstants.BLSPeedScale);
    setState(backRight, BRDriveMotor, BRSteerMotor, BRPID, BREncoder, Constants.OperatorConstants.BRSPeedScale);
    modulePositions = updateModulePos();
    odometry.update(getGyroPitch(), modulePositions);
    field.setRobotPose(odometry.getPoseMeters());
    		/* Prepare line to print */
		_sb.append("\tout:");
		/* Cast to int to remove decimal places */
		_sb.append((int) (FLDriveMotor.getMotorOutputPercent() * 100));
		_sb.append("%");	// Percent

		_sb.append("\tspd:");
		_sb.append(FLDriveMotor.getSelectedSensorVelocity(0));
		_sb.append("u"); 	// Native units
    Instrum.Process(FRDriveMotor, _sb);
    updateSmartDashboard();
  }
/**Gets the encoder value.
 * 
 * @param encoder The encoder to get the value of.
 * @return Rotation2D object of the encoder.
 */
  public Rotation2d getEncoderValue(Encoder encoder) {
    return Rotation2d.fromDegrees((encoder.get() * (1.0/1.2)));
  }
/**Updates the positions of the swerve modules.
 * 
 * @return An array of four updated SwerveModulePostions.
 */
  public SwerveModulePosition[] updateModulePos(){
    SwerveModulePosition[] Modules = new SwerveModulePosition[4];
    Modules[0] = new SwerveModulePosition(getDistanceTravelled(FLDriveMotor), getEncoderValue(FLEncoder));
    Modules[1] = new SwerveModulePosition(getDistanceTravelled(FRDriveMotor), getEncoderValue(FREncoder));
    Modules[2] = new SwerveModulePosition(getDistanceTravelled(BLDriveMotor), getEncoderValue(BLEncoder));
    Modules[3] = new SwerveModulePosition(getDistanceTravelled(BRDriveMotor), getEncoderValue(BREncoder));
    return Modules;
  }
/**Gets the distance travelled by a motor.
 * 
 * @param motor The motor to get the distance of.
 * @return The distance travelled in meters.
 */
  public double getDistanceTravelled(TalonFX motor){
    return (motor.getSelectedSensorPosition()/2048.0) * Constants.OperatorConstants.driveWheelRatio * (Math.PI * Units.inchesToMeters(4));
  }
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
/**Sets the wheels to new ModuleStates.
 * 
 * @param ModuleStates An array of four ModuelStates to set the wheels to.
 */
  public void setModuleStates(SwerveModuleState ModuleStates[]){
    setState(ModuleStates[0], FLDriveMotor, FLSteerMotor, FLPID, FLEncoder, Constants.OperatorConstants.FLSPeedScale);
    setState(ModuleStates[1], FRDriveMotor, FRSteerMotor, FRPID, FREncoder, Constants.OperatorConstants.FRSPeedScale);
    setState(ModuleStates[2], BLDriveMotor, BLSteerMotor, BLPID, BLEncoder, Constants.OperatorConstants.BLSPeedScale);
    setState(ModuleStates[3], BRDriveMotor, BRSteerMotor, BRPID, BREncoder, Constants.OperatorConstants.BRSPeedScale);
  }
  /** Field oriented driving with the joystick.
   * 
   * @param xSpeed Forwards and backwards speed in meters per second.
   * @param ySpeed Left and right speed in meters per second.
   * @param rot The rotation speed in meters per second.
   */
  public void driveWithMisery(double xSpeed, double ySpeed, double rot) {
    isZeroed = false;
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroPitch());
  }
/**Gets the yaw of the gyroscope.
 * 
 * @return The yaw of the gyroscope as a Rotation2D object.
 */
public Rotation2d getGyroPitch() {
  return Gyro.getRotation2d();
}
/**Gets the yaw of the gyroscope.
 * 
 * @return The pitch of the gyroscope as a Rotation2D object.
 */
public Rotation2d getGyroYaw() {
  return Rotation2d.fromDegrees(Gyro.getPitch()+180);
}
/**Sets the state of a swerve module.
 * 
 * @param state The SwerveModuleState object to mimic.
 * @param driveMotor The drive motor to set.
 * @param steerMotor The steer motot to set.
 * @param PID The PID.
 * @param encoder The steer encoder to read.
 * @param speedScale A value to offset wheel speeds.
 */
  public void setState(SwerveModuleState state, TalonFX driveMotor, TalonSRX steerMotor, PIDController PID, Encoder encoder, double speedScale) {
    steerMotor.set(ControlMode.PercentOutput, PID.calculate(getEncoderValue(encoder).getDegrees(), roundAngle(state.angle.getDegrees())));
    driveMotor.set(ControlMode.PercentOutput, (state.speedMetersPerSecond/MAX_ROBOT_SPEED)*speedScale*3);
  }
  /**Updates the smart dashboard
   * 
   */
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
    SmartDashboard.putData(field);
    SmartDashboard.putNumber("DISTANCE TRAVELLED", getDistanceTravelled(FLDriveMotor));
    
    // SmartDashboard.putNumber("RAW ENCODER: FL", FLEncoder.get());
    // SmartDashboard.putNumber("RAW ENCODER: FR", FREncoder.get());
    // SmartDashboard.putNumber("RAW ENCODER: BL", BLEncoder.get());
    // SmartDashboard.putNumber("RAW ENCODER: BR", BREncoder.get());
  }
/**Turns all motors to zero degrees and stops them.
 * 
 */
  public void zeroMotors(){
    frontLeft = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    frontRight = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    backLeft = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    backRight = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    isZeroed = true;
  }
/**Irrelevant yet necessary rounding thing.
 * 
 * @param angle 
 * @return Rounded angle.
 */
  public double roundAngle(double angle){
    return Math.round(angle * 1.2)/1.2;
  }
/**Temporary method used for tuning PID.
 * 
 */
  public void PIDTuner(){
    FLSteerMotor.set(ControlMode.PercentOutput, FLPID.calculate(getEncoderValue(FLEncoder).getDegrees(), 90));
    FRSteerMotor.set(ControlMode.PercentOutput, FRPID.calculate(getEncoderValue(FREncoder).getDegrees(), 90));
    BLSteerMotor.set(ControlMode.PercentOutput, BLPID.calculate(getEncoderValue(BLEncoder).getDegrees(), 90));
    BRSteerMotor.set(ControlMode.PercentOutput, BRPID.calculate(getEncoderValue(BREncoder).getDegrees(), 90));
  }

  public void driveForwards(double speed){
    FLDriveMotor.set(ControlMode.PercentOutput, speed);
    FRDriveMotor.set(ControlMode.PercentOutput, speed);
    BLDriveMotor.set(ControlMode.PercentOutput, speed);
    BRDriveMotor.set(ControlMode.PercentOutput, speed);
  }
/**Temporary method used for tuning speed.
 * 
 */
  public void SpeedTuner(){
    FLDriveMotor.set(ControlMode.PercentOutput, .5*Constants.OperatorConstants.FLSPeedScale);
    FRDriveMotor.set(ControlMode.PercentOutput, .5*Constants.OperatorConstants.FRSPeedScale);
    BLDriveMotor.set(ControlMode.PercentOutput, .5*Constants.OperatorConstants.BLSPeedScale);
    BRDriveMotor.set(ControlMode.PercentOutput, .5*Constants.OperatorConstants.BRSPeedScale);
  }
/**Resets odometry to a set position.
 * 
 * @param pose The postition to set the odometry to.
 */
  public void resetOdometery(Pose2d pose) {
    odometry.resetPosition(Gyro.getRotation2d(), modulePositions, pose);
  }

  public boolean gate = false;
  public boolean gamin = false;

  public boolean isEngaged(){
    if((Gyro.getRoll() < -5 || Gyro.getRoll() > 5))
      gate = true;
    if(gate == true)
      if((Gyro.getRoll() > -5 && Gyro.getRoll() < 5))
        gamin = true;
    return gamin;
  }

  public void resetEngagement(){
    gate = false;
    gamin = false;
  }

/**Resets the gyro to zero. 
 * 
 */
  public void resetGyro(){
    Gyro.reset();
  }

}
