package frc.robot.commands.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.RobotContainer;


public class DriveStraightTurn extends SequentialCommandGroup {
  Pose2d start = new Pose2d(0,0,new Rotation2d(0));
  //create new waypoints here
  List<Translation2d> waypoints = List.of(
                                    new Translation2d(1,0.0),
                                    new Translation2d(2, 0.0)
                                    // new Translation2d(7,0),
                                    // new Translation2d(6,0)
                                  );

  Pose2d end = new Pose2d(1, 0, Rotation2d.fromDegrees(-90));

  public DriveStraightTurn(RobotContainer robot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new WaitCommand(1));
    // TODO Mihai
    // addCommands(robot.createAutoNavigationCommand(start, waypoints, end));
  }
}
