package frc.robot.subsystems.leds;

public class LedConstants {
  // PSOC LED Codes:
  // 0 off
  // 1 cube
  // 2 cone
  // 3 blue and red
  // 4 Blue w/ wht burst
  // 5 Red w/ wht burst
  // 6 party
  // 7 right1
  // 8 right2
  // 9 right3
  // 10 center
  // 11 left3
  // 12 left2
  // 13 left1
  // 14 cylon / oposite rings
  // 15 rgb chase

  public static final int climbingLEDCommand = 4;
  public static final int hasCoralLEDCommand = 6;
  public static final int hasAlgaeLEDCommand = 10;
  public static final int autoLEDCommand = 3;
  public static final int preMatchLEDCommand = 15;
  public static final int disconnectedLEDCommand = 0;
  public static final int teleopLEDCommand = 5;

  // TODO: SET ACTUAL
  public static final int ledControllerBit0Port = 0;
  public static final int ledControllerBit1Port = 1;
  public static final int ledControllerBit2Port = 2;
  public static final int ledControllerBit3Port = 3;

  public static final int numLedCommands = 15;
}
