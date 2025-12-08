package frc.robot.subsystems.leds;

import frc.robot.subsystems.leds.Leds.LEDStates;
import org.littletonrobotics.junction.AutoLog;

public interface LedIO {
  @AutoLog
  public static class LEDIOInputs {
    public boolean transactionSuccess = true;
  }

  public default void setState(LEDStates state) {}
  ;

  public default void updateInputs(LEDIOInputs inputs) {}
}
