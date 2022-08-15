// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.List;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import frc.robot.constants.Ports;
import frc.robot.constants.Settings;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utills.CustomRamseteControllerAbstraction;
import frc.robot.commands.DrivetrainDrive;
import frc.robot.commands.auton.DriveStraight;
import frc.robot.commands.auton.DriveStraightTurn;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();

  public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
  private static SendableChooser<Command> autonChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(new DrivetrainDrive(drivetrain, driver));
    
    configureButtonBindings();
    configureAutons();
  }


  private void configureButtonBindings() {}


  public void configureAutons() {
    autonChooser.setDefaultOption("Drive Straight", new DriveStraight(this));
    autonChooser.setDefaultOption("Drive Straight and Turn", new DriveStraightTurn(this));
    SmartDashboard.putData("Autonomous", autonChooser);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }


  public Command createAutoNavigationCommand(Pose2d start, List<Translation2d> waypoints, Pose2d end) {
    System.out.print("Creating Auto Command, start time: ");
    System.out.println(Timer.getFPGATimestamp());
  
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Settings.Drivetrain.Motion.FeedForward.kS, Settings.Drivetrain.Motion.FeedForward.kV, Settings.Drivetrain.Motion.FeedForward.kA),
        Settings.Drivetrain.Motion.KINEMATICS, 10);
  
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(Settings.Drivetrain.MAX_SPEED_AUTON, Settings.Drivetrain.MAX_ACCEL_AUTON)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Settings.Drivetrain.Motion.KINEMATICS)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);
  
    // An example trajectory to follow. All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
    System.out.print("Generated Trajectory, start time: ");
    System.out.println(Timer.getFPGATimestamp());
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, this.drivetrain::getPose,
        new CustomRamseteControllerAbstraction(Settings.Drivetrain.Motion.RAMSETE_B, Settings.Drivetrain.Motion.RAMSETE_ZETA),
        Settings.Drivetrain.Motion.KINEMATICS,
        this.drivetrain::tankDriveVelocity, this.drivetrain);
  
    // Run path following command, then stop at the end.
    System.out.print("Finished Creating Auto Command, end time: ");
    System.out.println(Timer.getFPGATimestamp());
    return ramseteCommand.andThen(() -> this.drivetrain.stop());
  }
}



