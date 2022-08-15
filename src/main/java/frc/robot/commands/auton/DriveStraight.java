package frc.robot.commands.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.RobotContainer;


public class DriveStraight extends SequentialCommandGroup {
  Pose2d start = new Pose2d(0,0,new Rotation2d(0));

  List<Translation2d> waypoints = List.of(
                                    new Translation2d(1,0.0)
                                    // new Translation2d(5,1),
                                    // new Translation2d(7,0),
                                    // new Translation2d(6,0)
                                  );

  Pose2d end = new Pose2d(1, 0, new Rotation2d(0));

  public DriveStraight(RobotContainer robot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new WaitCommand(5));
    addCommands(robot.createAutoNavigationCommand(start, waypoints, end));
  }
}