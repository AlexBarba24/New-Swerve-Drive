// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AprilTagReader;
import frc.robot.subsystems.Drivetrain;

public class DrivingCommand extends CommandBase {
  Drivetrain myDrivetrain;
  DoubleSupplier myJoyX;
  DoubleSupplier myJoyY;
  DoubleSupplier myJoyX2;
  double slowMo = 1;
  /**
   * Command to swerve drive.
   * 
   * @param drivetrain a drivetrain object
   * @param joyX supplier for the left joystick X
   * @param joyY supplier for the left joystick Y
   * @param joyX2 supplier for the right joystick X
   */
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
    double autoAim;
    if(RobotContainer.driveController.getYButton())
      autoAim = AprilTagReader.aim();
    else
      autoAim = 0;
    double xSpeed = 0;
    double ySpeed = 0;
    double radSpeed = 0;
    
    if(RobotContainer.driveController.getAButton()){
      slowMo = .8;
    }
    if(RobotContainer.driveController.getBButton()){
      slowMo = 0.3;
    }
      
    if(!(RobotContainer.driveController.getXButton())){
      if(!(myJoyX.getAsDouble() > -0.5 && myJoyX.getAsDouble() < 0.5 && myJoyY.getAsDouble() > -0.5 && myJoyY.getAsDouble() < 0.5)) {
        xSpeed = Constants.OperatorConstants.driveSpeedScale * myJoyX.getAsDouble() * Constants.OperatorConstants.maxSpeed * slowMo;
        ySpeed = Constants.OperatorConstants.driveSpeedScale * myJoyY.getAsDouble() * Constants.OperatorConstants.maxSpeed * slowMo;
      }
      if(!(myJoyX2.getAsDouble() > -0.5 && myJoyX2.getAsDouble() < 0.5)){
        radSpeed = Constants.OperatorConstants.rotationSpeedScale * myJoyX2.getAsDouble() * Constants.OperatorConstants.driveSpeedScale * slowMo;
      }
      myDrivetrain.driveWithMisery(ySpeed, xSpeed, radSpeed-autoAim);
    }else {
      myDrivetrain.resetGyro();
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myDrivetrain.driveWithMisery(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
