// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ManualTurret extends CommandBase {
  DoubleSupplier rotation;
  TurretSubsystem turret;
  /** Creates a new ManualTurret. */
  public ManualTurret(DoubleSupplier rotation, TurretSubsystem turretSubsystem) {
    addRequirements(turretSubsystem);
    this.rotation = rotation;
    this.turret = turretSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setTurretSpeed(rotation.getAsDouble(), true);
  }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    turret.setTurretSpeed(0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
