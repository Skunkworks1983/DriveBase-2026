package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.subsystems.leds.Leds.LEDStates;

public class LedIOReal implements LedIO {
  public final I2C ledController = new I2C(Port.kOnboard, LedConstants.ledControllerI2CAdress);
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
        return (int) LedConstants.autoLEDCommand.get();
      case HAS_ALGAE:
        return (int) LedConstants.hasAlgaeLEDCommand.get();
      case HAS_CORAL:
        return (int) LedConstants.hasCoralLEDCommand.get();
      case PRE_MATCH:
        return (int) LedConstants.preMatchLEDCommand.get();
      case CLIMBING:
        return (int) LedConstants.climbingLEDCommand.get();
      default:
        return (int) LedConstants.disconnectedLEDCommand.get();
    }
  }

  @Override
  public void setState(LEDStates state) {
    int command = getLEDCommand(state) | LedConstants.bitmask; // Default bit mask 0x00 (none)
    byte[] i2cData = new byte[] {(byte) command};

    sendSuccess = !ledController.writeBulk(i2cData, 1); // write bulk returns if the send is aborted; this is inverted so logging displays the correct color to indicate a failure
  }

  @Override
  public void updateInputs(LEDIOInputs inputs) {
    inputs.transactionSuccess = sendSuccess;
  }
}
