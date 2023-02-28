// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

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
      winchMotor.set(ControlMode.PercentOutput, 0.3);
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
    updateSmartDashboard();
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
