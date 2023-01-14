// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DrivingCommand extends CommandBase {
  Drivetrain myDrivetrain;
  DoubleSupplier myJoyX;
  DoubleSupplier myJoyY;
  DoubleSupplier myJoyX2;

  /** Creates a new DrivingCommand. */
  public DrivingCommand(Drivetrain drivetrain, DoubleSupplier joyX, DoubleSupplier joyY, DoubleSupplier joyX2) {
    myDrivetrain = drivetrain;
    myJoyX = joyX;
    myJoyY = joyY;
    myJoyX2 = joyX2;
    addRequirements(myDrivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(myJoyX.getAsDouble() > 0.5) {
      double xSpeed = Constants.OperatorConstants.driveSpeedScale * myJoyX.getAsDouble();
      double ySpeed = Constants.OperatorConstants.driveSpeedScale * myJoyY.getAsDouble();
      double radSpeed = Constants.OperatorConstants.rotationSpeedScale * myJoyX2.getAsDouble();
      myDrivetrain.driveWithMisery(xSpeed, ySpeed, radSpeed);
    } else {
      myDrivetrain.driveWithMisery(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
