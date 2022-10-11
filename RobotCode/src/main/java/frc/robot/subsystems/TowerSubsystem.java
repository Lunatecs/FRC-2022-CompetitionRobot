// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TowerConstants;



public class TowerSubsystem extends SubsystemBase {
  private TalonSRX frontMotor = new TalonSRX(TowerConstants.FRONT_TOWER_MOTOR_ID);
  private VictorSPX backMotor = new VictorSPX(TowerConstants. BACK_TOWER_MOTOR_ID);



  /** Creates a new TowerSubsystem. */
  public TowerSubsystem() {
    backMotor.configFactoryDefault();
    frontMotor.configFactoryDefault();
    frontMotor.setNeutralMode(NeutralMode.Brake);
    backMotor.setNeutralMode(NeutralMode.Brake);
    backMotor.follow(frontMotor);
    backMotor.setInverted(false);
    frontMotor.setInverted(false);

  }

  public void runTower(double speed){

    frontMotor.set(ControlMode.PercentOutput, speed);


  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
