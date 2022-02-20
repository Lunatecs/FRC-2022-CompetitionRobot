// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private TalonSRX intake = new TalonSRX(IntakeConstants.FORWARD_MOTOR_ID);
  
  /** Creates a new IntakeSubsystem. */

  public IntakeSubsystem() {

    intake.configFactoryDefault();
    intake.setNeutralMode(NeutralMode.Brake);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggleIntake() {

  }

  public void runIntake(double speed) {
    intake.set(ControlMode.PercentOutput, speed);
  }
}
