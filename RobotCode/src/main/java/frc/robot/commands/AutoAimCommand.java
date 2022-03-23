// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TrackingConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.Date;

public class AutoAimCommand extends CommandBase {
  LimelightSubsystem limelight;
  TurretSubsystem turret;
  PIDController pidController = new PIDController(0.03, TrackingConstants.kI, TrackingConstants.kD);
  
  double scanSpeed = 0.2;
  long start = 0;
  /** Creates a new AutoAimCommand. */
  public AutoAimCommand(TurretSubsystem turret, LimelightSubsystem limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.limelight = limelight;
    this.pidController.setSetpoint(0);
    this.pidController.setTolerance(.3);
    addRequirements(turret);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    this.start = new Date().getTime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.isValidTarget()) {
      turret.setTurretSpeed(this.pidController.calculate(limelight.getTX()), true);
    }
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    return limelight.isOnTarget() && new Date().getTime() > start + 2000;
  }
}
