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

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {
    mLift.setNeutralMode(NeutralModeValue.Brake);
    // mLift.setPosition(0);
    // eLift.reset();
  }

  private boolean liftCycle = false;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("LiftSwitch", !wLiftMax.get());
    SmartDashboard.putBoolean("LiftCycle", liftCycle);
    SmartDashboard.putNumber("lift position", eLift.get());
    if (!wLiftMax.get()) {
      eLift.reset();
    }
  }

  private TalonFX mIntake = new TalonFX(10);
  private TalonFX mLift = new TalonFX(15);
  public DigitalInput wLiftMax = new DigitalInput(4);
  public Encoder eLift = new Encoder(2, 3);

  public void intakeZero() {
    mIntake.set(0.0);
  }

  public void intakeForward() {
    mIntake.set(-1.0);
  }

  public void intakeReverse() {
    mIntake.set(1.0);
  }

  public void liftZero() {
    mLift.set(0);
  }

  public void liftOut() {
    mLift.set(0.15);
  }

  public void liftOut4() {
    mLift.set(0.4);
  }

  public void liftIn() {
    if(liftCycle) {
      mLift.set(-0.15);
    }
  }

  public void modLiftCycle() {
    liftCycle = true;
  }
}
