// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ArcadeDriveCommand extends CommandBase {

  DrivetrainSubsystem drivetrain;
  DoubleSupplier speedSupplier;
  DoubleSupplier rotationSupplier;
  BooleanSupplier slow;
  BooleanSupplier reverse;

    /** Creates a new ArcadeDriveCommand. */
  public ArcadeDriveCommand(DrivetrainSubsystem drivetrain, DoubleSupplier speedSupplier, DoubleSupplier rotationSupplier, BooleanSupplier slow, BooleanSupplier reverse) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.speedSupplier = speedSupplier;
    this.rotationSupplier = rotationSupplier;
    this.slow = slow;
    this.reverse =  reverse;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumber("rotation", rotationSupplier.getAsDouble());
    //SmartDashboard.putNumber("speed", speedSupplier.getAsDouble());

    // The slow button has been changed to a fast button
    double speedMulti = .66;
    if(slow.getAsBoolean()) {
      speedMulti=1.0;
    }
    if(reverse.getAsBoolean()) {
      speedMulti = speedMulti * -1.0;
    }
    drivetrain.arcadeDrive(speedSupplier.getAsDouble()*speedMulti, rotationSupplier.getAsDouble()*speedMulti);
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
