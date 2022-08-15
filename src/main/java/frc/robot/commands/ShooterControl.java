// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Settings;
import frc.robot.subsystems.Shooter;

public class ShooterControl extends CommandBase {
  private final Shooter shooter;
  private final Gamepad driver;
  private final IStream commandedSpeed;


  /** Creates a new ShooterSetRPM. */
  public ShooterControl(Shooter shooter, Gamepad driver) {
    this.shooter = shooter;
    this.driver = driver;

    this.commandedSpeed =
        IStream.create(() -> driver.getLeftTrigger())
                .filtered(
                        x -> SLMath.map(x, 0, 1, Settings.Shooter.MIN_SPEED.get(), 
                                        Settings.Shooter.MAX_SPEED.get()),
                        x -> SLMath.spow(x, Settings.Shooter.SPEED_POWER.get()),
                        new LowPassFilter(Settings.Shooter.SPEED_FILTER));
                        
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setRPM(-commandedSpeed.get(),commandedSpeed.get());
    //SmartDashboard.putNumber("Debug/Shooter Setpoint", commandedSpeed.get());
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
