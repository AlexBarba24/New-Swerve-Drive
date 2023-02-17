// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmCommand extends CommandBase {
  Translation2d point;
  BooleanSupplier isDone;
  boolean retracted;
  Arm m_Arm;
  /** Creates a new ArmCommand. */
  public ArmCommand(Arm arm, Translation2d point, BooleanSupplier theLean) {
    // Use addRequirements() here to declare subsystem dependencies.
    retracted = true;
    isDone = theLean;
    this.point = point;
    m_Arm = arm;
    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(retracted == false){
      if(!m_Arm.extendArm(Units.inchesToMeters(28.5)))
        return;
      else
        retracted = true;
    }
    if(m_Arm.turnToPoint(point))
      System.out.println("**********************************************************");
    m_Arm.extendArm(point);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.STOP_NOW();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !isDone.getAsBoolean();
  }
}
