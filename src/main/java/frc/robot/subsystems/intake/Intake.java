// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

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

    SmartDashboard.putNumber("intake setter", -0.8);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("LimitSwitch", !wLiftMax.get());
    SmartDashboard.putNumber("lift position", eLift.get());
    if (!wLiftMax.get()) {
      eLift.reset();
    }
  }

  private TalonFX mIntake = new TalonFX(IntakeConstants.kIntakeMotorId);
  private TalonFX mLift = new TalonFX(IntakeConstants.kLiftMotorId);
  public DigitalInput wLiftMax = new DigitalInput(IntakeConstants.kLiftSwitchId);
  public Encoder eLift = new Encoder(2, 3);

  public void intakeForward() {
    mIntake.set(-0.2); //TODO: Find optimal intake speed.
  }

  public void intakeZero() {
    mIntake.set(0.0);
  }

  public void liftZero() {
    mLift.set(0);
  }

  public void liftOut() {
    mLift.set(0.2);
  }

  public void liftIn() {
    mLift.set(-0.2);
  }

  // VVVVV Methods below are for testing VVVVV

  // public void LiftOut() {
  //   if (!wLiftMax.get()) {
  //     mLift.set(0);
  //     eLift.reset();
  //   } else {
  //     mLift.set(0.2);
  //   }
  // }

  // public void LiftIn() {
  //   if (eLift.get() > 700) {
  //     mLift.set(0);
  //   }
  //   else {
  //     mLift.set(-0.2);
  //   }
  // }

  public void liftDown() {
    mLift.set(0.05);
  }

  public void liftUp() {
    if (eLift.get() > 700) {
      mLift.set(0);
    }
    else {
      mLift.set(-0.1);
    }
  }

  public void liftDown1() {
    if (!wLiftMax.get()) {
      mLift.set(0);
      eLift.reset();
    } else {
      mLift.set(0.1);
    }
  }

  // public void intakeNumMethod() {
  //   mIntake.set(SmartDashboard.getNumber("intake setter", -0.0)); // Make it negative.
  // }

  public void intakeNumMethod() {
    mIntake.set(-0.8);
  }
}
