// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */
  private final TalonSRX turret = new TalonSRX(TurretConstants.Turret_ID);
  private NeutralMode TURRET_NEUTRALMODE = NeutralMode.Brake;
  private PIDController pidControllerFwd;
  private PIDController pidControllerBck;
  private DigitalInput limitSwitch = new DigitalInput(TurretConstants.LIMIT_SWITCH_PORT_2);
  
  public TurretSubsystem() {
    turret.configFactoryDefault();
    turret.setNeutralMode(TURRET_NEUTRALMODE);
    resetPosition();
    pidControllerFwd = new PIDController(TurretConstants.FwdKp,TurretConstants.FwdKi,TurretConstants.FwdKd);
    pidControllerFwd.setSetpoint(TurretConstants.FwdMaxSensorPostion);
    pidControllerBck = new PIDController(TurretConstants.BckKp,TurretConstants.BckKi,TurretConstants.BckKd);
    pidControllerBck.setSetpoint(TurretConstants.BckMaxSensorPostion);
    turret.setSensorPhase(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Encoder", getPosition());
    SmartDashboard.putBoolean("Turret limit switch", getLimitSwitch());
    super.periodic();
  }



  public void setTurretSpeed(double speed, boolean ignoreMin) {
    double position = getPosition(); 
    double speedLimitFwd = pidControllerFwd.calculate(position);
    double speedLimitBck = pidControllerBck.calculate(position);
    SmartDashboard.putNumber("speedLimitFwd", speedLimitFwd);
    SmartDashboard.putNumber("speedLimitBck", speedLimitBck);
    
    if(speed > speedLimitFwd && speed > 0) {
      speed = speedLimitFwd;
    } else if(speed < speedLimitBck && speed <= 0) {
      speed = speedLimitBck;
    }
    if (!(speed == 0) && Math.abs(speed) < TurretConstants.MinSpeed && !ignoreMin) {
      if(speed < 0) {
        speed = -TurretConstants.MinSpeed;
      } else if(speed > 0) {
        speed = TurretConstants.MinSpeed;
      }
    }
    SmartDashboard.putNumber("actualSpeed", speed);
    turret.set(ControlMode.PercentOutput, speed);
  }


  public double getPosition() {
    return this.turret.getSelectedSensorPosition(0);
  }

  public void resetPosition() {
    turret.setSelectedSensorPosition(0, 0, 10);
  }

  public boolean getLimitSwitch() {
    return !limitSwitch.get();
  }

  public boolean isFwdLimit() {
    if (TurretConstants.FwdMaxSensorPostion-1500 <= getPosition()) {
      return true;
    }
    else {
      return false;
    }
  }

  public boolean isRevLimit() {
    if (TurretConstants.BckMaxSensorPostion+1500 >= getPosition()) {
      return true;
    }
    else {
      return false;
    }
  }
    

}
