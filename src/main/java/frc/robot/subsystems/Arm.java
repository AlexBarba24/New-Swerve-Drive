// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Instrum;

public class Arm extends SubsystemBase {

	StringBuilder _sb = new StringBuilder();


  //Gear ratio of 49:1 for the falcon
  public TalonFX winchMotor = new TalonFX(Constants.OperatorConstants.WINCH_MOTOR_ID);
  public TalonSRX pivotMotor = new TalonSRX(Constants.OperatorConstants.PIVOT_MOTOR_ID);

  //Gives value between 0 - 360
  public Encoder encoder = new Encoder(8, 9);

  PIDController winchPID = new PIDController(Constants.OperatorConstants.winchPID[0], Constants.OperatorConstants.winchPID[1], Constants.OperatorConstants.winchPID[2]);
  PIDController armPID = new PIDController(Constants.OperatorConstants.armPID[0], Constants.OperatorConstants.armPID[1], Constants.OperatorConstants.armPID[2]);
  
  private double armLength = 0;
  private double armAngle = 0;

  /** Creates a new Arm. */
  public Arm() {
    winchMotor.setInverted(true);
    winchMotor.setNeutralMode(NeutralMode.Brake);
    pivotMotor.setNeutralMode(NeutralMode.Brake);
    winchMotor.setSelectedSensorPosition(0);
    //test code
    		/* Factory default hardware to prevent unexpected behavior */
		winchMotor.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		winchMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		winchMotor.configNeutralDeadband(0.001, 30);

		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		winchMotor.setSensorPhase(false);
		winchMotor.setInverted(true);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // winchMotor.setSensorPhase(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		winchMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
		winchMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

		/* Set the peak and nominal outputs */
		winchMotor.configNominalOutputForward(0, 30);
		winchMotor.configNominalOutputReverse(0, 30);
		winchMotor.configPeakOutputForward(1, 30);
		winchMotor.configPeakOutputReverse(-1, 30);

		/* Set Motion Magic gains in slot0 - see documentation */
		winchMotor.selectProfileSlot( 0, 0);
		winchMotor.config_kF(0, 0.04507600793, 30);
		winchMotor.config_kP( 0, 0.1261405672, 30);
		winchMotor.config_kI( 0, 0.001, 30);
		winchMotor.config_kD( 0, 1.261405672, 30);

		/* Set acceleration and vcruise velocity - see documentation */
		winchMotor.configMotionCruiseVelocity(17021, 30);
		winchMotor.configMotionAcceleration(17021.25, 30);

		/* Zero the sensor once on robot boot up */
		winchMotor.setSelectedSensorPosition(0, 0, 30);
    
    		/* Factory default hardware to prevent unexpected behavior */
		pivotMotor.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
	
		pivotMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.None , 0, 30);
		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		pivotMotor.configNeutralDeadband(0.001, 30);

		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		pivotMotor.setSensorPhase(false);
		pivotMotor.setInverted(false);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // pivotMotor.setSensorPhase(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		pivotMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
		pivotMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

		/* Set the peak and nominal outputs */
		pivotMotor.configNominalOutputForward(0, 30);
		pivotMotor.configNominalOutputReverse(0, 30);
		pivotMotor.configPeakOutputForward(1, 30);
		pivotMotor.configPeakOutputReverse(-1, 30);

		/* Set Motion Magic gains in slot0 - see documentation */
		pivotMotor.selectProfileSlot(0, 0);
		pivotMotor.config_kF(0, 0, 30);
		pivotMotor.config_kP(0, 0, 30);
		pivotMotor.config_kI(0, 0, 30);
		pivotMotor.config_kD(0, 0, 30);

		/* Set acceleration and vcruise velocity - see documentation */
		pivotMotor.configMotionCruiseVelocity(15000, 30);
		pivotMotor.configMotionAcceleration(6000, 30);

		/* Zero the sensor once on robot boot up */
		pivotMotor.setSelectedSensorPosition(0, 0, 30);
  }

  public boolean retractArm(){
    winchMotor.set(ControlMode.PercentOutput, winchPID.calculate(winchEncoderReadingMeters(), 0));
    return winchEncoderReadingMeters() < 0.1;
  }

  public boolean extendArm(Translation2d setpoint){
    calculate(setpoint);
    winchMotor.set(ControlMode.PercentOutput, winchPID.calculate(winchEncoderReadingMeters(), armLength));
    // System.out.println(winchPID.calculate(winchEncoderReadingMeters(), armLength));
    return (armLength - 0.1 <  winchEncoderReadingMeters() && armLength + 0.1 > encoder.get());
  }
  public boolean extendArm(double length){
    armLength = length;
    winchMotor.set(ControlMode.PercentOutput, winchPID.calculate(winchEncoderReadingMeters(), length));
    // System.out.println(winchPID.calculate(winchEncoderReadingMeters(), armLength));
    return (armLength - 0.1 <  winchEncoderReadingMeters() && armLength + 0.1 > encoder.get());
  }

  public boolean turnToPoint(Translation2d setpoint){
    calculate(setpoint);
    pivotMotor.set(ControlMode.PercentOutput, -armPID.calculate(getEncoderValue(), armAngle));
    System.out.println(-armPID.calculate(getEncoderValue(), armAngle));
    return (armAngle - 3 <  encoder.get() && armAngle + 3 > encoder.get());
  }
  public boolean turnToPoint(double angle){
    armAngle = angle;
    pivotMotor.set(ControlMode.PercentOutput, -armPID.calculate(getEncoderValue(), armAngle));
    System.out.println(-armPID.calculate(getEncoderValue(), armAngle));
    return (armAngle - 10 <  getEncoderValue() && armAngle + 10 > getEncoderValue());
  }
 
  public void extendArmManual(){
    if(winchEncoderReadingMeters() < 10)
      winchMotor.set(ControlMode.PercentOutput, 1);
  }

  public void retractArmManual(){
    if(winchEncoderReadingMeters() > 0.05)
      winchMotor.set(ControlMode.PercentOutput, -0.3);
  }
  public void retractArmManualManual(){
    winchMotor.set(ControlMode.PercentOutput, -0.3);
  }

  public void armWithMisery(double joyInput){
    double sign = 0;
    if(joyInput > 0)
      sign = 1;
    else
      sign = -1;
    double value = joyInput * joyInput * sign;
    pivotMotor.set(ControlMode.PercentOutput, value);
  }

  public double getEncoderValue(){
    return encoder.getDistance()/7.5;
  }

  public void calculate(Translation2d setpoint){
    double x = setpoint.getX();
    double y = setpoint.getY();
    double r = Math.sqrt((x*x)+(y*y));
    armLength = r;
    armAngle = (-Math.acos(y/r)+Math.PI) * (180/Math.PI);
    //106.7 degrees 
  }

  public void STOP_NOW(){
    winchMotor.set(ControlMode.PercentOutput, 0);
    pivotMotor.set(ControlMode.PercentOutput, 0);
  }

  public double winchEncoderReadingMeters(){
    return (winchMotor.getSelectedSensorPosition()/2048/49 * Math.PI * Units.inchesToMeters(1.125));
  }


  @Override
  public void periodic() {
    // double motorOutput = winchMotor.getMotorOutputPercent();
    // _sb.append("\tOut%:");
		// _sb.append(motorOutput);
		// _sb.append("\tVel:");
		// _sb.append(winchMotor.getSelectedSensorVelocity(0));
    // _sb.append("\tPosition:");
    // _sb.append(winchMotor.getSelectedSensorPosition());
    // Instrum.Process(winchMotor, _sb);

		// pivotMotor.setSelectedSensorPosition(encoder.getDistance(), 0, 30);


    // _sb.append("\tOut%:");
		// _sb.append(pivotMotor.getMotorOutputPercent());
		// _sb.append("\tVel:");
		// _sb.append(pivotMotor.getSelectedSensorVelocity(0));
    // _sb.append("\tPosition:");
    // _sb.append(pivotMotor.getSelectedSensorPosition());

    // Instrum.Process(pivotMotor, _sb);

    updateSmartDashboard();


  }

  public void motionMagicWinch(double targetPos){

    //MinPos = 0
    //Max Pos = 33400

    winchMotor.set(TalonFXControlMode.MotionMagic, targetPos);

    /* Append more signals to print when in speed mode */
    _sb.append("\terr:");
    _sb.append(winchMotor.getClosedLoopError(0));
    _sb.append("\ttrg:");
    _sb.append(targetPos);
  }

  public void resetEncoder(){
    winchMotor.setSelectedSensorPosition(0);
  }

  public void updateSmartDashboard(){
    SmartDashboard.putNumber("Arm Length", winchEncoderReadingMeters());
    SmartDashboard.putNumber("Arm Angle", getEncoderValue());
    SmartDashboard.putNumber("Target Angle", armAngle);
    SmartDashboard.putNumber("Target Length", armLength);
    
  }

}
