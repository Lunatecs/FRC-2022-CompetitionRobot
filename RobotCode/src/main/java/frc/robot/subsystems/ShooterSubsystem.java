// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  TalonFX shooterLeader = new TalonFX(ShooterConstants.SHOOTER_LEADER);
  TalonFX shooterFollower = new TalonFX(ShooterConstants.SHOOTER_FOLLOWER);

  public ShooterSubsystem() {
    shooterLeader.configFactoryDefault();
    shooterFollower.configFactoryDefault();

    shooterFollower.follow(shooterLeader);

    shooterFollower.setInverted(true);

    shooterFollower.setNeutralMode(NeutralMode.Coast);
    shooterLeader.setNeutralMode(NeutralMode.Coast);

    shooterLeader.setSensorPhase(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ;
    SmartDashboard.putNumber("Shooter Velocity", shooterLeader.getSelectedSensorVelocity());
  }

  public void toggleHood() {/* ur mom */}
  
  public void setShooterSpeed(double speed) {
    shooterLeader.set(ControlMode.PercentOutput, speed);
  }

  public void setShooterVelocity(double velocity) {
    if(velocity==0.0) {
      shooterLeader.config_kP(0, 0);
    } else {
      shooterLeader.config_kF(0, .07);
      shooterLeader.config_kP(0, .2);
    }
    shooterLeader.set(ControlMode.Velocity, velocity);
  }

}
