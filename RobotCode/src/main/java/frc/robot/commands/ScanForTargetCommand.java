// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TrackingConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import javax.xml.namespace.QName;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ScanForTargetCommand extends CommandBase {
  /** Creates a new ScanForTargetCommand. */
  double scanSpeed = 0.75;

  TurretSubsystem turret;
  LimelightSubsystem limelight;

  PIDController pController = new PIDController(TrackingConstants.kP, TrackingConstants.kI, TrackingConstants.kD);

  public ScanForTargetCommand(TurretSubsystem turret, LimelightSubsystem limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.limelight = limelight;
    pController.setSetpoint(0);
    addRequirements(turret);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setLED(LimelightConstants.forceOn_ID);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 0.0;
    SmartDashboard.putNumber("TX",limelight.getTX());
    if (limelight.isValidTarget()) {
      speed = pController.calculate(limelight.getTX(), 0);
      turret.setTurretSpeed(speed, true);
    }
    else {
      if (turret.isFwdLimit()) {
        scanSpeed = -0.4;

      }
      else if (turret.isRevLimit()) {
        scanSpeed = 0.4;
      }
      turret.setTurretSpeed(scanSpeed, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.setLED(LimelightConstants.forceOff_ID);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
