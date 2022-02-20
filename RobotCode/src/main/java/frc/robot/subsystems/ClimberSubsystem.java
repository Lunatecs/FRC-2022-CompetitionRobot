// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  //private final DoubleSolenoid climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.FORWARD_CHANNEL, ClimberConstants.REVERSE_CHANNEL);
  
  private final TalonFX leftWinchMotor = new TalonFX(ClimberConstants.WINCH_MOTOR_ID);
  private final TalonFX rightWinchMotor = new TalonFX(ClimberConstants.WITCH_MOTOR_ID);
  private static NeutralMode CLIMBER_NEUTRALMODE = NeutralMode.Brake;

  public ClimberSubsystem() {
    leftWinchMotor.configFactoryDefault();
    rightWinchMotor.configFactoryDefault();
    leftWinchMotor.setNeutralMode(CLIMBER_NEUTRALMODE);
    rightWinchMotor.setInverted(true);
    leftWinchMotor.setInverted(false);
    rightWinchMotor.follow(leftWinchMotor);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runClimber(double speed) {
    leftWinchMotor.set(ControlMode.PercentOutput, speed);
  }

  public void toggleSolenoid() {
    //climberSolenoid.toggle();

  }
}
