// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.stuypulse.stuylib.control.PIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import frc.robot.constants.Ports;
import frc.robot.constants.Settings;
import frc.robot.constants.Settings.Drivetrain.Motion;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utills.CustomRamseteControllerAbstraction;
import frc.robot.commands.DrivetrainDrive;
import frc.robot.commands.auton.DoNothing;
import frc.robot.commands.auton.DriveStraight;
import frc.robot.commands.auton.DriveStraightTurn;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();

  public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
  private static SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    drivetrain.setDefaultCommand(new DrivetrainDrive(drivetrain, driver));
    // System.out.println("testtesttesttesttesttesttesttesttesttesttesttesttesttesttesttesttest");
    configureButtonBindings();
    configureAutons();
  }

  private void configureButtonBindings() {
  }

  public void configureAutons() {
    autonChooser.setDefaultOption("Drive Straight", new DriveStraight(this));
    // switch back to correct default
    autonChooser.addOption("Do Nothing", new DoNothing());
    autonChooser.addOption("Drive Straight and Turn", new DriveStraightTurn(this));
    SmartDashboard.putData("Driver Settings/Auto Chooser", autonChooser);
  }

  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast

    var autoVoltageConstraint =

        new DifferentialDriveVoltageConstraint(

            new SimpleMotorFeedforward(

                Motion.FeedForward.kS,

                Motion.FeedForward.kV,

                Motion.FeedForward.kA),

            Motion.KINEMATICS,

            10);

    // Create config for trajectory

    TrajectoryConfig config =

        new TrajectoryConfig(

            Settings.Drivetrain.MAX_SPEED_AUTON,

            Settings.Drivetrain.MAX_ACCEL_AUTON)

                // Add kinematics to ensure max speed is actually obeyed

                .setKinematics(Settings.Drivetrain.Motion.KINEMATICS)

                // Apply the voltage constraint

                .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.

    Trajectory exampleTrajectory =

        TrajectoryGenerator.generateTrajectory(

            // Start at the origin facing the +X direction

            new Pose2d(0, 0, new Rotation2d(0)),

            // Pass through these two interior waypoints, making an 's' curve path

            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),

            // End 3 meters straight ahead of where we started, facing forward

            new Pose2d(3, 0, new Rotation2d(0)),

            // Pass config

            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            drivetrain::getPose,
            new RamseteController(Settings.Drivetrain.Motion.RAMSETE_B, Settings.Drivetrain.Motion.RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                Motion.FeedForward.kS,
                Motion.FeedForward.kV,
                Motion.FeedForward.kA),
            Motion.KINEMATICS,
            drivetrain::getWheelSpeeds,
            new PIDController(Settings.Drivetrain.Motion.PID.kP, 0, 0),
            new PIDController(Settings.Drivetrain.Motion.PID.kD, 0, 0),
            // RamseteCommand passes volts to the callback
            drivetrain::tankDriveVolts,
            drivetrain
          );

    // Reset odometry to the starting pose of the trajectory.

    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.

    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }

  // public Command createAutoNavigationCommand(Pose2d start, List<Translation2d>
  // waypoints, Pose2d end) {
  // System.out.print("Creating Auto Command, start time: ");
  // System.out.println(Timer.getFPGATimestamp());
  // //return getAutonomousCommand();
  // var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
  // new SimpleMotorFeedforward(Settings.Drivetrain.Motion.FeedForward.kS,
  // Settings.Drivetrain.Motion.FeedForward.kV,
  // Settings.Drivetrain.Motion.FeedForward.kA),
  // Settings.Drivetrain.Motion.KINEMATICS, 10);
  // System.out.println("autoVoltageConstraint complete");
  // // Create config for trajectory
  // TrajectoryConfig config = new
  // TrajectoryConfig(Settings.Drivetrain.MAX_SPEED_AUTON,
  // Settings.Drivetrain.MAX_ACCEL_AUTON)
  // // Add kinematics to ensure max speed is actually obeyed
  // .setKinematics(Settings.Drivetrain.Motion.KINEMATICS)
  // // Apply the voltage constraint
  // .addConstraint(autoVoltageConstraint);

  // // An example trajectory to follow. All units in meters.
  // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start,
  // waypoints, end, config);
  // System.out.print("Generated Trajectory, start time: ");
  // System.out.println(Timer.getFPGATimestamp());
  // RamseteCommand ramseteCommand = new RamseteCommand(trajectory,
  // this.drivetrain::getPose,
  // new CustomRamseteControllerAbstraction(Settings.Drivetrain.Motion.RAMSETE_B,
  // Settings.Drivetrain.Motion.RAMSETE_ZETA),
  // Settings.Drivetrain.Motion.KINEMATICS, this.drivetrain::tankDriveVelocity,
  // this.drivetrain);

  // // Run path following command, then stop at the end.
  // System.out.print("Finished Creating Auto Command, end time: ");
  // System.out.println(Timer.getFPGATimestamp());
  // var refSpeed = CustomRamseteControllerAbstraction.

  // return
  // }
}
