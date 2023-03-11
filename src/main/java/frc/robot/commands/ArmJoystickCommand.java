// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

public class ArmJoystickCommand extends CommandBase {
  Arm arm;
  DoubleSupplier joyInput;
  DoubleSupplier getRightTriggerAxis;
  DoubleSupplier getLeftTriggerAxis;
  public Arm m_arm;
  /** Creates a new ArmJoystickCommand. */
  public ArmJoystickCommand(Arm arm, DoubleSupplier joyInput, DoubleSupplier getRightTriggerAxis, DoubleSupplier getLeftTriggerAxis) {
    this.arm = arm;
    this.joyInput = joyInput;
    this.getRightTriggerAxis = getRightTriggerAxis;
    this.getLeftTriggerAxis = getLeftTriggerAxis;
    m_arm = arm;
    addRequirements(m_arm);
    //addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joyWithDeadband;
    
    if(joyInput.getAsDouble() < 0.2 && joyInput.getAsDouble() > -0.2)
      joyWithDeadband = 0;
    else
      joyWithDeadband = joyInput.getAsDouble();
    m_arm.armWithMisery(joyWithDeadband);
      
    if(getRightTriggerAxis.getAsDouble() > 0.2 )
      m_arm.extendArmManual();
    else if(getLeftTriggerAxis.getAsDouble() > 0.2)
      m_arm.retractArmManual();
    else
      m_arm.winchMotor.set(ControlMode.PercentOutput, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //arm.STOP_NOW();
    m_arm.STOP_NOW();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void updateSmartDashboard(){
    SmartDashboard.putNumber("R_Trig_Axes", getRightTriggerAxis.getAsDouble());
    SmartDashboard.putNumber("Left Trigger Axis", getLeftTriggerAxis.getAsDouble());
  }
}
