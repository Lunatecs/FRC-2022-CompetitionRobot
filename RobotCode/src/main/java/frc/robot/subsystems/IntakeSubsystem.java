// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private TalonSRX frontMotor = new TalonSRX(IntakeConstants.FORWARD_MOTOR_ID);
  //commented out because it doesn't do anything right now anyway
  //private TalonSRX backMotor = new TalonSRX(IntakeConstants.BACK_MOTOR_ID);

  // I don't quite know if I am doing this right.
  //private DoubleSolenoid frontIntake = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.FORWARD_CHANNEL, IntakeConstants.REVERSE_CHANNEL);
  //private DoubleSolenoid backIntake = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.FORWARD_CHANNEL_2, IntakeConstants.REVERSE_CHANNEL_2);
  
  /** Creates a new IntakeSubsystem. */

  public IntakeSubsystem() {

    frontMotor.configFactoryDefault();
    //backMotor.configFactoryDefault();

    //backMotor.setNeutralMode(NeutralMode.Brake);
    frontMotor.setNeutralMode(NeutralMode.Brake);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggleIntake() {
    //frontIntake.toggle();
    //backIntake.toggle();
  }

  public void runIntake(double speed) {
    frontMotor.set(ControlMode.PercentOutput, speed);
  }
}
