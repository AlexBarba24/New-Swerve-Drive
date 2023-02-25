// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

public class ArmJoystickCommand extends CommandBase {
  Arm arm;
  DoubleSupplier joyInput;
  /** Creates a new ArmJoystickCommand. */
  public ArmJoystickCommand(Arm arm, DoubleSupplier joyInput) {
    this.arm = arm;
    this.joyInput = joyInput;
    addRequirements(arm);
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
    arm.armWithMisery(joyWithDeadband);
  
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.STOP_NOW();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
