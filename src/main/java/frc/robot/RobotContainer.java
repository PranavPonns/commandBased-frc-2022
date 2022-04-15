// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.common.Controller;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drive;
  private CommandScheduler m_commandScheduler;
  private final Controller joy;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_commandScheduler = CommandScheduler.getInstance();
    m_drive = Drivetrain.getInstance();
    joy = new Controller(0);
    
    m_commandScheduler.setDefaultCommand(m_drive, new Drive(m_drive, joy::getRawY, joy::getRawZ));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    
    var autoVoltageConstraint = 
    new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        DriveConstants.ksVolts,
        DriveConstants.kvVoltSecondsPerMeter,
        DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        10);
    
      TrajectoryConfig config = 
      new TrajectoryConfig(
        AutoConstants.maxVelocityMetersPerSecond, 
        AutoConstants.maxAccelerationMetersPerSecondSq)
      .setKinematics(DriveConstants.kDriveKinematics)
      .addConstraint(autoVoltageConstraint);
      
      Trajectory traj = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0, new Rotation2d(0)), 
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)), 
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

        RamseteCommand ramseteCommand = new RamseteCommand(
          traj,
          m_drive::getPose,
          new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), 
          new SimpleMotorFeedforward(DriveConstants.ksVolts,
          DriveConstants.kvVoltSecondsPerMeter,
          DriveConstants.kaVoltSecondsSquaredPerMeter), 
          DriveConstants.kDriveKinematics, 
          m_drive::getWheelSpeeds,
          new PIDController(DriveConstants.kP, 0, 0), 
          new PIDController(DriveConstants.kP, 0, 0), 
          m_drive::setVolts,
          m_drive
        );

        m_drive.resetOdometry(traj.getInitialPose());

        return ramseteCommand.andThen(() -> m_drive.drive(0,0));
  }
}
