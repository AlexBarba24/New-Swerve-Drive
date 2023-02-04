// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  //Gear ratio of 49:1 for the falcon
  public TalonFX winchMotor;
  public TalonSRX pivotMotor;

  //Gives value between 0 - 360
  public Encoder encoder;

  PIDController einchPID;
  PIDController armPID;
  
  private double armLength = 0;
  private double armAngle = 0;

  /** Creates a new Arm. */
  public Arm() {
    
  }

  public void turnToPoint(Translation2d setpoint){
    calculate(setpoint);

  }

  public void calculate(Translation2d setpoint){
    double x = setpoint.getX();
    double y = setpoint.getY();
    double r = Math.sqrt((x*x)+(y*y));
    armLength = r;
    armAngle = (-Math.acos(y/r)+Math.PI) * (180/Math.PI);
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
