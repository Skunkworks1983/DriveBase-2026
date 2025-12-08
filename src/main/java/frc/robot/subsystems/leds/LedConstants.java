package frc.robot.subsystems.leds;

import frc.robot.util.LoggedTunableNumber;

public class LedConstants {
  public static final int psocI2CAdress = 0x20;
  public static final int bitmask = 0x00;
  public static LoggedTunableNumber climbingLEDCommand =
      new LoggedTunableNumber("LEDS/Climbing Command", 0xFF);
  public static LoggedTunableNumber hasCoralLEDCommand =
      new LoggedTunableNumber("LEDS/Climbing Command", 0x0F);
  public static LoggedTunableNumber hasAlgaeLEDCommand =
      new LoggedTunableNumber("LEDS/Climbing Command", 0xF0);
  public static LoggedTunableNumber autoLEDCommand =
      new LoggedTunableNumber("LEDS/Climbing Command", 0xAA);
  public static LoggedTunableNumber preMatchLEDCommand =
      new LoggedTunableNumber("LEDS/Climbing Command", 0x01);
  public static LoggedTunableNumber disconnectedLEDCommand =
      new LoggedTunableNumber("LEDS/Climbing Command", 0x55);
  public static LoggedTunableNumber teleopLEDCommand =
      new LoggedTunableNumber("LEDS/Climbing Command", 0x00);
}
