package frc.robot.subsystems.leds;

import java.util.HashMap;
import java.util.HashSet;

import frc.robot.subsystems.leds.Leds.LEDStates;

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

  // TODO: SET ACTUAL
  public static final int ledControllerBit0Port = 0;
  public static final int ledControllerBit1Port = 1;
  public static final int ledControllerBit2Port = 2;
  public static final int ledControllerBit3Port = 3;

  public static final int numLedCommands = 15;


  // Hashmap tying each led state to an integer representing its command
  public static HashMap<LEDStates, Integer> ledCommands = new HashMap<>(){
    {
        put(LEDStates.DISCONNECTED, 0);
        put(LEDStates.PRE_MATCH, 15);
        put(LEDStates.TELEOP, 10);
        put(LEDStates.AUTO, 3);
        put(LEDStates.HAS_ALGAE, 4);
        put(LEDStates.HAS_CORAL, 5);
        put(LEDStates.CLIMBING, 6);

    }
  };
}
