// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {

  private final CollectorIO collectorMotorOne;
  private final CollectorIO collectorMotorTwo;
  private final CollectorIOInputsAutoLogged inputs = new CollectorIOInputsAutoLogged();

  public Collector(CollectorIO collectorMotorOne, CollectorIO collectorMotorTwo) {
    this.collectorMotorOne = collectorMotorOne;
    this.collectorMotorTwo = collectorMotorTwo;
  }

  // motor ID 11

  @Override
  public void periodic() {
    collectorMotorOne.updateInputs(inputs);
    collectorMotorTwo.updateInputs(inputs);
  }

  public void setCollectorVelocity(double motorOneVelocity, double motorTwoVelocity) {
    collectorMotorOne.setCollectorVelocity(motorOneVelocity);
    collectorMotorTwo.setCollectorVelocity(motorTwoVelocity);
  }
}
