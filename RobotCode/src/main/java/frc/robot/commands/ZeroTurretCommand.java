// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroTurretCommand extends PIDCommand {
  private TurretSubsystem turretSubsystem;
  /** Creates a new ZeroTurretCommand. */
  public ZeroTurretCommand(TurretSubsystem turretSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(0.0001, 0, 0),
        // This should return the measurement
        () -> turretSubsystem.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          turretSubsystem.setTurretSpeed(output, true);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.turretSubsystem = turretSubsystem;
    addRequirements(turretSubsystem);
    getController().setTolerance(100);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.setTurretSpeed(0, true);
  }
}
