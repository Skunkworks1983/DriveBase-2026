package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.subsystems.leds.Leds.LEDStates;

public class LedIOReal implements LedIO {
  private final DigitalOutput ledBit0 = new DigitalOutput(LedConstants.ledControllerBit0Port);
  private final DigitalOutput ledBit1 = new DigitalOutput(LedConstants.ledControllerBit1Port);
  private final DigitalOutput ledBit2 = new DigitalOutput(LedConstants.ledControllerBit2Port);
  private final DigitalOutput ledBit3 = new DigitalOutput(LedConstants.ledControllerBit3Port);
  private boolean sendSuccess = false;

  /**
   * @implNote Gets from the tunable number network table a hex representing an led state (e.g.
   *     0xF0)
   * @param state LED state to get hex command for
   * @return Hex literal from logged tunable numbers
   */
  private int getLEDCommand(LEDStates state) {
    switch (state) {
      case AUTO:
        return LedConstants.autoLEDCommand;
      case HAS_ALGAE:
        return LedConstants.hasAlgaeLEDCommand;
      case HAS_CORAL:
        return LedConstants.hasCoralLEDCommand;
      case PRE_MATCH:
        return LedConstants.preMatchLEDCommand;
      case CLIMBING:
        return LedConstants.climbingLEDCommand;
      default:
        return LedConstants.disconnectedLEDCommand;
    }
  }

  @Override
  public void setState(LEDStates state) {
    int command = getLEDCommand(state);

    // Invert the command bc the PSOC inverts it back
    int invertedCommand = LedConstants.numLedCommands - command;

    // Converting the command int to binary for the DIO
    ledBit0.set((invertedCommand & 1) != 0);
    ledBit1.set((invertedCommand & 2) != 0);
    ledBit2.set((invertedCommand & 4) != 0);
    ledBit3.set((invertedCommand & 8) != 0);
  }

  @Override
  public void updateInputs(LEDIOInputs inputs) {
    inputs.transactionSuccess = sendSuccess;
  }
}
