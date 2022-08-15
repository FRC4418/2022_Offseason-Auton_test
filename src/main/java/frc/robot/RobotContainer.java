// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import frc.robot.constants.Ports;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.AutonHalt;
import frc.robot.commands.DrivetrainDrive;
import frc.robot.commands.FeederControl;
import frc.robot.commands.FeederReverse;
import frc.robot.commands.ShooterControl;
import frc.robot.commands.ShooterReverse;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final Shooter shooter = new Shooter();
  private final Feeder feeder = new Feeder();

  private final AutonHalt autonHalt = new AutonHalt();


  public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(new DrivetrainDrive(drivetrain, driver));
    shooter.setDefaultCommand(new ShooterControl(shooter, driver));
    feeder.setDefaultCommand(new FeederControl(feeder, driver));
    configureButtonBindings();
  }


  private void configureButtonBindings() {
    driver.getRightBumper().whenHeld(new FeederReverse(feeder));
    driver.getLeftBumper().whenHeld(new ShooterReverse(shooter));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autonHalt;
  }
}
