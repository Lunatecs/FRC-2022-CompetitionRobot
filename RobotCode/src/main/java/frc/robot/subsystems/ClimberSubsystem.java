// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣤⣤⣤⣤⣤⣶⣦⣤⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀ 
⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⣿⡿⠛⠉⠙⠛⠛⠛⠛⠻⢿⣿⣷⣤⡀⠀⠀⠀⠀⠀ 
⠀⠀⠀⠀⠀⠀⠀⠀⣼⣿⠋⠀⠀⠀⠀⠀⠀⠀⢀⣀⣀⠈⢻⣿⣿⡄⠀⠀⠀⠀ 
⠀⠀⠀⠀⠀⠀⠀⣸⣿⡏⠀⠀⠀⣠⣶⣾⣿⣿⣿⠿⠿⠿⢿⣿⣿⣿⣄⠀⠀⠀ 
⠀⠀⠀⠀⠀⠀⠀⣿⣿⠁⠀⠀⢰⣿⣿⣯⠁⠀⠀⠀⠀⠀⠀⠀⠈⠙⢿⣷⡄⠀ 
⠀⠀⣀⣤⣴⣶⣶⣿⡟⠀⠀⠀⢸⣿⣿⣿⣆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣷⠀ 
⠀⢰⣿⡟⠋⠉⣹⣿⡇⠀⠀⠀⠘⣿⣿⣿⣿⣷⣦⣤⣤⣤⣶⣶⣶⣶⣿⣿⣿⠀ 
⠀⢸⣿⡇⠀⠀⣿⣿⡇⠀⠀⠀⠀⠹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠃⠀ 
⠀⣸⣿⡇⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠉⠻⠿⣿⣿⣿⣿⡿⠿⠿⠛⢻⣿⡇⠀⠀ 
⠀⣿⣿⠁⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀  ⢸⣿⣧⠀⠀ 
⠀⣿⣿⠀⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀  ⢸⣿⣿⠀⠀ 
⠀⣿⣿⠀⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀  ⢸⣿⣿⠀⠀ 
⠀⢿⣿⡆⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀  ⢸⣿⡇⠀⠀ 
⠀⠸⣿⣧⡀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀  ⣿⣿⠃⠀⠀ 
⠀⠀⠛⢿⣿⣿⣿⣿⣇⠀⠀⠀⠀⠀⣰⣿⣿⣷⣶⣶⣶⣶⠶  ⢠⣿⣿⠀⠀⠀ 
⠀⠀⠀⠀⠀⠀⠀⣿⣿⠀⠀⠀⠀⠀⣿⣿⡇⠀⣽⣿⡏⠁⠀⠀  ⢸⣿⡇⠀⠀⠀ 
⠀⠀⠀⠀⠀⠀⠀⣿⣿⠀⠀⠀⠀⠀⣿⣿⡇⠀⢹⣿⡆⠀⠀⠀  ⣸⣿⠇⠀⠀⠀ 
⠀⠀⠀⠀⠀⠀⠀⢿⣿⣦⣄⣀⣠⣴⣿⣿⠁⠀⠈⠻⣿⣿⣿⣿⡿⠏⠀⠀⠀⠀ 
⠀⠀⠀⠀⠀⠀⠀⠈⠛⠻⠿⠿⠿⠿⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
*/
package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private final DoubleSolenoid climberSolenoidRight = new DoubleSolenoid(
    PneumaticsModuleType.REVPH,
    ClimberConstants.FORWARD_CHANNEL_R, 
    ClimberConstants.REVERSE_CHANNEL_R);

  private final DoubleSolenoid climberSolenoidLeft = new DoubleSolenoid(
    PneumaticsModuleType.REVPH,
    ClimberConstants.FORWARD_CHANNEL_L, 
    ClimberConstants.REVERSE_CHANNEL_L);
  
  private final TalonFX leftWinchMotor = new TalonFX(ClimberConstants.WINCH_MOTOR_ID);
  private final TalonFX rightWinchMotor = new TalonFX(ClimberConstants.WITCH_MOTOR_ID);
  private static NeutralMode CLIMBER_NEUTRALMODE = NeutralMode.Brake;

  private final DigitalInput rightLimitSwitch = new DigitalInput(ClimberConstants.LIMIT_SWITCH_PORT_1);
  private final DigitalInput leftLimitSwitch = new DigitalInput(ClimberConstants.LIMIT_SWITCH_PORT);

  //private boolean isLimitTriggered = false;

  public ClimberSubsystem() {

    leftWinchMotor.configFactoryDefault();
    rightWinchMotor.configFactoryDefault();

    leftWinchMotor.setNeutralMode(CLIMBER_NEUTRALMODE);

    rightWinchMotor.setInverted(true);
    leftWinchMotor.setInverted(false);

    //rightWinchMotor.follow(leftWinchMotor);

    climberSolenoidLeft.set(DoubleSolenoid.Value.kForward);
    //climberSolenoidRight.set(DoubleSolenoid.Value.kForward);

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Right Limit Triggered", rightLimitSwitch.get());
    SmartDashboard.putBoolean("Left Limit Triggered", leftLimitSwitch.get());
    SmartDashboard.putNumber("Left Encoder", leftWinchMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Encoder", rightWinchMotor.getSelectedSensorPosition());
    // This method will be called once per scheduler run
  }

  public void runClimber(double speed) {
    runClimberLeft(speed);
    runClimberRight(speed);
  }

  public void runClimberLeft(double speed) {
    if(speed < 0) {
      leftWinchMotor.set(ControlMode.PercentOutput, speed);
    } else {
      if (!leftLimitSwitch.get()) {
        leftWinchMotor.set(ControlMode.PercentOutput, 0);
        leftWinchMotor.setSelectedSensorPosition(0);
      } else {
        leftWinchMotor.set(ControlMode.PercentOutput, speed);
      }
    }
  }

  public void runClimberRight(double speed) {
    if(speed < 0) {
      rightWinchMotor.set(ControlMode.PercentOutput, speed);
    } else {
      if (!rightLimitSwitch.get()) {
        rightWinchMotor.set(ControlMode.PercentOutput, 0);
        rightWinchMotor.setSelectedSensorPosition(0);
      } else {
        rightWinchMotor.set(ControlMode.PercentOutput, speed);
      }
    }
  }

  public void resetClimberEncoders() {
    rightWinchMotor.setSelectedSensorPosition(0);
    leftWinchMotor.setSelectedSensorPosition(0);
  }

  public double getRightClimberEncoder() {
    return rightWinchMotor.getSelectedSensorPosition();
  }

  public double getLeftClimberEncoder() {
    return leftWinchMotor.getSelectedSensorPosition();
  }

  public double getAverageClimberEncoder(){
    return (getRightClimberEncoder()+getLeftClimberEncoder())/2.0;
  }

  public void toggleSolenoid() {
    //this.climberSolenoidRight.toggle();
    this.climberSolenoidLeft.toggle();

  }
}
