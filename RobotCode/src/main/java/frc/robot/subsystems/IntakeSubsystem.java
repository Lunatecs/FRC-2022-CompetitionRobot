// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private TalonSRX frontMotor = new TalonSRX(IntakeConstants.FORWARD_MOTOR_ID);
  private TalonSRX backMotor = new TalonSRX(IntakeConstants.BACK_MOTOR_ID);

  private final DoubleSolenoid frontIntake = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, 
    IntakeConstants.FORWARD_CHANNEL_F, 
    IntakeConstants.REVERSE_CHANNEL_F);

    private final DoubleSolenoid backIntake = new DoubleSolenoid(
      PneumaticsModuleType.REVPH, 
      IntakeConstants.FORWARD_CHANNEL_B, 
      IntakeConstants.REVERSE_CHANNEL_B);

  //commented out because it doesn't do anything right now anyway
  
 
  /** Creates a new IntakeSubsystem. */

  public IntakeSubsystem() {

    frontMotor.configFactoryDefault();
    backMotor.configFactoryDefault();

    backMotor.setNeutralMode(NeutralMode.Brake);
    frontMotor.setNeutralMode(NeutralMode.Brake);
    //backMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, periodMs, timeoutMs)

    // PLACEHOLDER VALUES FIX THIS BEFORE COMPETITION!!!
    frontIntake.set(DoubleSolenoid.Value.kReverse);
    backIntake.set(DoubleSolenoid.Value.kReverse);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggleFrontIntake() {
    frontIntake.toggle();
  }

  public void toggleBackIntake() {
    backIntake.toggle();
  }

  public void runIntake(double speed) {
    frontMotor.set(ControlMode.PercentOutput, speed);
    backMotor.set(ControlMode.PercentOutput, speed);
  }

  public void intakeOut() {
    frontIntake.set(DoubleSolenoid.Value.kForward);
  }
}
