// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
  /** Creates a new Lift. */
  public Lift() {
    mLift.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("LiftSwitch", !wLiftMax.get());
    SmartDashboard.putBoolean("LiftCycle", liftCycle);
    SmartDashboard.putNumber("LiftPos", eLift.get());
    if (!wLiftMax.get()) {
      eLift.reset();
    }
  }

  private TalonFX mLift = new TalonFX(15);
  public DigitalInput wLiftMax = new DigitalInput(4);
  public Encoder eLift = new Encoder(2, 3);

  private boolean liftCycle = false;

  public void liftZero() {
    mLift.set(0);
  }

  public void liftOut() {
    mLift.set(0.25);
  }

  public void liftOutFast() {
    mLift.set(0.5);
  }

  public void liftIn() {
    if(liftCycle) {
      mLift.set(-0.25);
    }
  }

  public void liftInEmergency() {
    mLift.set(-0.1);
  }

  public void modLiftCycle() {
    liftCycle = true;
  }
}
