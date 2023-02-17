// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FailSafeAuto extends SequentialCommandGroup {
  
  /** Creates a new FailSafeAuto. */
  public FailSafeAuto(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(drivetrain);

    addCommands(
      new ParallelDeadlineGroup(
        new WaitCommand(5), 
        new InstantCommand(()->drivetrain.driveForwards(.1),drivetrain)
      ),
      new InstantCommand(()->drivetrain.driveForwards(0), drivetrain)
    );
  }
}
