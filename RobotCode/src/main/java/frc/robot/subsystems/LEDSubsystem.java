// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Comparator;
import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private Spark ledControl = new Spark(0);

  private static final double SOLID_GREEN = 0.77;
  private static final double SOLID_RED = 0.61;  
  private static final double SOLID_BLUE = 0.87;
  private static final double BLINKING_RED = -0.11;
  private static final double BLINKING_BLUE = -0.09;
  private static final double RAINBOW_GLITTER = -0.89;
  private static final double DEFAULT_COLOR = -0.57;//0.57;
  

  private PriorityQueue<PriorityColor> queue = null;

  public final PriorityColor DEFAULT = new PriorityColor(DEFAULT_COLOR, 100);//100

  public final PriorityColor ON_TARGET = new PriorityColor(SOLID_GREEN, 20);//20

  public final PriorityColor ONE_BLUE = new PriorityColor(SOLID_BLUE, 40);//40
  public final PriorityColor BOTH_BLUE = new PriorityColor(BLINKING_BLUE, 30);//30

  // First alarm fires 35 seconds to the end of the match
  // Second alarm fires 25 seconds to the end of the match
  public final PriorityColor FIRST_ALARM = new PriorityColor(SOLID_RED, 90);
  public final PriorityColor SECOND_ALARM = new PriorityColor(BLINKING_RED, 80);

  public final PriorityColor CLIMB = new PriorityColor(RAINBOW_GLITTER, 10);

  private static LEDSubsystem ledSubsystem = null;
  
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    queue = new PriorityQueue<PriorityColor>(new PriorityColor());
    addDefaultColor();
  }

  public static LEDSubsystem instanceOf() {
    return ledSubsystem;
  }

  public static void setLEDSubsystem(LEDSubsystem led) {
    ledSubsystem = led;
  }

  public void addColor(PriorityColor color) {
    if(!queue.contains(color)) {
      queue.add(color);
    }
  }

  public void removeColor(PriorityColor color) {
    if(queue.contains(color)) {
      queue.remove(color);
    }
  }

  private void addDefaultColor() {
    queue.add(DEFAULT);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putString("LED Value", queue.peek().color+"");
    ledControl.set(queue.peek().color);
    //SmartDashboard.putBoolean("LED Spark", ledControl.isAlive());
    //SmartDashboard.putData(ledControl);
  }

  public class PriorityColor implements Comparator<PriorityColor>, Comparable<PriorityColor> {

    public double color;
    public int priority;
    public int loopCount = -10;

    public PriorityColor() {}

    public PriorityColor(double color, int priority) {
      this.color = color;
      this.priority = priority;
    }

    public int compare(PriorityColor p1 , PriorityColor p2) {
        if(p1.priority>= p2.priority) {
          return 1;
        } else {
          return -1;
        }
    }

    public int compareTo(PriorityColor p2) {
      return compare(this, p2);
    }
  }

}
