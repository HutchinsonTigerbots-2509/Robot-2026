// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
  /** Creates a new Lift. */

  private CurrentLimitsConfigs limit = new CurrentLimitsConfigs().withStatorCurrentLimit(10);

  public Lift() {
    mLiftA.setNeutralMode(NeutralModeValue.Brake);
    mLiftB.setNeutralMode(NeutralModeValue.Brake);
    mLiftA.getConfigurator().apply(limit);
    mLiftB.getConfigurator().apply(limit);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("LiftSwitch", !wLiftMax.get());
    SmartDashboard.putNumber("LiftPos", eLift.get());
    if (!wLiftMax.get()) {
      eLift.reset();
    }
  }

  private TalonFX mLiftA = new TalonFX(15);
  private TalonFX mLiftB = new TalonFX(16);
  
  public DigitalInput wLiftMax = new DigitalInput(4);
  public Encoder eLift = new Encoder(2, 3);

  private boolean liftCycle = false;

  public void liftZero() {
    mLiftA.set(0);
    mLiftB.set(0);
  }

  public void liftOut() {
    mLiftA.set(0.25);
    mLiftB.set(-0.25);
  }

  public void liftOutFast() {
    mLiftA.set(0.8);
    mLiftB.set(-0.8);
  }

  public void liftIn() {
    if(liftCycle) {
      mLiftA.set(-0.25);
      mLiftB.set(0.25);
    }
  }

  public void liftInFast() {
    if(liftCycle) {
      mLiftA.set(-0.4);
      mLiftB.set(0.4);
    }
  }

  public void liftInEmergency() {
    mLiftA.set(-0.1);
    mLiftB.set(0.1);
  }

  public void modLiftCycle() {
    liftCycle = true;
  }
}
