// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private int kIntakeTorqueMax = 140;

  private int intakeTorqueCount = 0;
  private int intakeTorqueCount1 = 0;
  private int intakeTorqueCount2 = 0;

  public Intake() {
    mIntakeA.setNeutralMode(NeutralModeValue.Brake);
    mIntakeB.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("mIntakeATorque", mIntakeA.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putNumber("mIntakeBTorque", mIntakeB.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putNumber("mIntakeJamResolution", intakeJamResolution());
    SmartDashboard.putNumber("IntakeTorqueCount", intakeTorqueCount);
    SmartDashboard.putNumber("IntakeTorqueCount1", intakeTorqueCount1);
    SmartDashboard.putNumber("IntakeTorqueCount2", intakeTorqueCount2);
  }

  private TalonFX mIntakeA = new TalonFX(13);
  private TalonFX mIntakeB = new TalonFX(14);

  public void intakeZero() {
    mIntakeA.set(0.0);
    mIntakeB.set(0.0);
  }

  public void intakeForward() {
    mIntakeA.set(1.0);
    mIntakeB.set(-1.0);

    // mIntakeA.set(intakeJamResolution());
    // mIntakeB.set(-1 * intakeJamResolution());
  }

  public void intakeShoot() {
    mIntakeA.set(0.25);
    mIntakeB.set(-0.25);
  }

  public void intakeReverse() {
    mIntakeA.set(-1.0);
    mIntakeB.set(1.0);
  }

  private double intakeJamResolution() {
    if (intakeJamDetector() && intakeTorqueCount2 <= 0) {
      return -1.0;
    } else {
      intakeTorqueCount1++;
      intakeTorqueCount2--;
      return 1.0;
    }
  } 

  public boolean intakeJamDetector() {
    if (Math.abs(mIntakeA.getTorqueCurrent().getValueAsDouble()) > kIntakeTorqueMax || Math.abs(mIntakeB.getTorqueCurrent().getValueAsDouble()) > kIntakeTorqueMax) {
      intakeTorqueCount++;
      if (intakeTorqueCount > 50) {
        intakeTorqueCount2 = 100;
        return true;
      } else {
        return false;
      }
    } else { 
      intakeTorqueCount = 0;
      return false;
    }
  }
}
