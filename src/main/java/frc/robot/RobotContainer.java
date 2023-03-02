// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmCommandEpicer;
import frc.robot.commands.ArmExtendCommand;
import frc.robot.commands.ArmExtendCommandDANGER;
import frc.robot.commands.ArmJoystickCommand;
import frc.robot.commands.ArmRetractCommand;
import frc.robot.commands.DrivingCommand;
import frc.robot.commands.EngaginCommand;
import frc.robot.commands.FailSafeAuto;
import frc.robot.subsystems.AprilTagReader;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Drivetrain drivetrain = new Drivetrain();
  AprilTagReader aprilTagReader = new AprilTagReader();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static XboxController driveController = new XboxController(0);
  public static CommandXboxController operatorController = new CommandXboxController(1);
  public static Claw claw = new Claw();
  public static Arm arm = new Arm();

  public void updateSmartDashboard()
  {
    SmartDashboard.putData(arm);
  }
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    drivetrain.setDefaultCommand(new DrivingCommand(drivetrain, 
                                                    ()->driveController.getLeftX(), 
                                                    ()->driveController.getLeftY(), 
                                                    ()->driveController.getRightX()));
    arm.setDefaultCommand(new ArmJoystickCommand(arm,
                                                ()->operatorController.getLeftY(), 
                                                ()->operatorController.getRightTriggerAxis(),
                                                ()->operatorController.getLeftTriggerAxis()));
    

    // arm.setDefaultCommand(new ArmCommand(arm));
    
    operatorController.a().toggleOnTrue(new InstantCommand(() -> claw.solenoidToggle()));
    System.out.print("got here");
    //operatorController.rightTrigger().toggleOnTrue(new ArmExtendCommand(arm));
    //operatorController.leftTrigger().toggleOnTrue(new ArmRetractCommand(arm));
    operatorController.x().toggleOnTrue(new ArmExtendCommandDANGER(arm));
    operatorController.y().toggleOnTrue(new ArmCommandEpicer(arm, 0, Units.inchesToMeters(28.5), ()->operatorController.y().getAsBoolean()));
    

    //Max arm length: .967 meters
    //arm tower is 45 inches from ground
    //6 inches from front bumper

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }
      
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws IOException
   */
  public Command getAutonomousCommand() {

    final String engagedAuto = "Engage in autonomous";
    final String pathPlanner = "Path following auto";

    SendableChooser<String> m_chooser = new SendableChooser<>();

    SmartDashboard.putData("Auto Chooser", m_chooser);

    m_chooser.addOption(engagedAuto, engagedAuto);
    m_chooser.addOption(pathPlanner, pathPlanner);

    PathPlannerTrajectory testPath = PathPlanner.loadPath("New Path", new PathConstraints(Constants.OperatorConstants.kMaxSpeerMetersPerSecond, Constants.OperatorConstants.kMaxAccelerationMetersPerSecondSquared), true);
  
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));

    drivetrain.resetOdometery(testPath.getInitialPose());
    drivetrain.resetGyro();
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drivetrain::getPose, // Pose2d supplier
      drivetrain::resetOdometery, // Pose2d consumer, used to reset odometry at the beginning of auto
      drivetrain.m_kinematics, // SwerveDriveKinematics
      new PIDConstants(Constants.OperatorConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(Constants.OperatorConstants.kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      drivetrain::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
  );

    // 2. Generate trajectory
  //   String TrajectoryJSON = "paths/output/TestAuto.wpilib.json";
  //   Trajectory trajectory;
  //   try {
  //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(TrajectoryJSON);
  //     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  //  } catch (IOException ex) {
  //     DriverStation.reportError("Unable to open trajectory: " + Filesystem.getDeployDirectory().toPath().resolve(TrajectoryJSON) , ex.getStackTrace());
  //     return null;
  //  }
     //PIDController xController = new PIDController(Constants.OperatorConstants.kPXController, 0, 0);
     //PIDController yController = new PIDController(Constants.OperatorConstants.kPYController, 0, 0);
    //ProfiledPIDController thetaController = new ProfiledPIDController(Constants.OperatorConstants.kPThetaController, 0, 0, Constants.OperatorConstants.kThetaControllerConstraints);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
     String trajectoryJSON = "paths/TestAuto.wpilib.json";
     TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.OperatorConstants.kMaxSpeerMetersPerSecond, Constants.OperatorConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(drivetrain.m_kinematics);
     // ADD NEW AUTO WAYPOINTS HERE
     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(
                                        // TESTING AUTONOMOUS X WAYPOINTS
                                          new Pose2d(0, 0, new Rotation2d(0)),
                                          new Pose2d(0.25, 0, new Rotation2d(0)),
                                          new Pose2d(0.5, 0, new Rotation2d(0)),
                                          new Pose2d(0.75, 0, new Rotation2d(0)),
                                          new Pose2d(1, 0, new Rotation2d(0)),
                                          new Pose2d(1.25, 0, new Rotation2d(0)),
                                          new Pose2d(1.5, 0, new Rotation2d(0)),
                                          new Pose2d(1.75, 0, new Rotation2d(0)),
                                          new Pose2d(2, 0, new Rotation2d(0))),
                                        // TESTING AUTONOMOUS Y WAYPOINTS
                                        //   new Pose2d(2, 0.25, new Rotation2d(0)),
                                        //   new Pose2d(2, 0.5, new Rotation2d(0)),
                                        //   new Pose2d(2, 0.75, new Rotation2d(0)),
                                        //   new Pose2d(2, 1, new Rotation2d(0)),
                                        // // TESTING AUTONOMOUS ROTATION
                                        //   new Pose2d(2, 1, new Rotation2d(Math.PI * 0.5)),
                                        //   new Pose2d(2, 1, new Rotation2d(Math.PI * 1.0)),
                                        //   new Pose2d(2, 1, new Rotation2d(Math.PI * 1.5)),
                                        //   new Pose2d(2, 1, new Rotation2d(Math.PI * 2.0))),
                                          trajectoryConfig);
    
        // try {      // } catch (IOException ex) {
        //    DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        // }

     PIDController xController = new PIDController(Constants.OperatorConstants.kPXController, 0, 0);
     PIDController yController = new PIDController(Constants.OperatorConstants.kPXController, 0, 0);
     ProfiledPIDController thetaController = new ProfiledPIDController(Constants.OperatorConstants.kPThetaController, 0, 0, Constants.OperatorConstants.kThetaControllerConstraints);
  //   // An example command will be run in autonomous
    // Command autonomousCommand = new SwerveControllerCommand(trajectory, drivetrain::getPose, drivetrain.m_kinematics, xController, yController, thetaController, drivetrain::setModuleStates, drivetrain);
     Command autoCommand = new SwerveControllerCommand(trajectory, drivetrain::getPose, drivetrain.m_kinematics, xController, yController, thetaController, drivetrain::setModuleStates, drivetrain);
     return new SequentialCommandGroup(new InstantCommand(() -> drivetrain.resetOdometery(trajectory.getInitialPose())),autoCommand,new InstantCommand(() -> drivetrain.zeroMotors()));
     //switch (m_chooser.getSelected()) {
       //case pathPlanner:
        //return autoBuilder.fullAuto(testPath);
         //return new SequentialCommandGroup(autoBuilder.fullAuto(testPath),autoBuilder.fullAuto(testPath),autoBuilder.fullAuto(testPath));
       //case engagedAuto:
        // return new EngaginCommand(drivetrain);
      // return new FailSafeAuto(drivetrain);
     
        // default:
      //   return autoBuilder.fullAuto(testPath);
 //   }
 
    // return new FailSafeAuto(drivetrain);
  }
}
