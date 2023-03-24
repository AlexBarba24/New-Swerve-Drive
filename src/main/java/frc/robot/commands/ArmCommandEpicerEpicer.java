// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class ArmCommandEpicerEpicer extends CommandBase {
  double angle;
  double length;
  boolean retracted;
  BooleanSupplier isDone;
  Arm m_Arm;
  Claw m_Claw;
  /** Creates a new ArmCommand. */
  public ArmCommandEpicerEpicer(Claw claw, Arm arm, double angle, double length, BooleanSupplier isDoneDone) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    this.length = length;
    m_Arm = arm;
    m_Claw = claw;
    retracted = true;
    isDone = isDoneDone;
    addRequirements(m_Arm, m_Claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_Claw.solenoidToggle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(retracted == false){
    //   if(!m_Arm.extendArm(Units.inchesToMeters(28.5)))
    //     return;
    //   else
    //     retracted = true;
    // }
    if(m_Arm.turnToPoint(angle))
       m_Arm.motionMagicWinch(length);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_Claw.solenoidToggle();
    m_Arm.STOP_NOW();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !isDone.getAsBoolean();
  }
}
