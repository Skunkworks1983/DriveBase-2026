package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Leds extends SubsystemBase {
  private final LEDIOInputsAutoLogged ledInputs = new LEDIOInputsAutoLogged();
  private final LedIO ledIO;

  public static enum LEDStates {
    CLIMBING,
    HAS_CORAL,
    HAS_ALGAE,
    AUTO,
    PRE_MATCH,
    DISCONNECTED
  }

  public Leds(LedIO io) {
    ledIO = io;
  }

  @Override
  public void periodic() {
    ledIO.updateInputs(ledInputs);
    Logger.processInputs("Leds", ledInputs);
  }

  public void setLedStae(LEDStates state) {
    Logger.recordOutput("Leds/Desired State", state);
    ledIO.setState(state);
  }
}
