// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constLED;
import frc.robot.RobotMap.mapLED;

public class LED extends SubsystemBase {
  private static LED instance;
  private CANdle LED;

  /** Creates a new LED. */

  private LED() {
    LED = new CANdle(mapLED.LED_CAN);
    LED.configAllSettings(constLED.LED_CONFIG);
  }

  public static LED getInstance() {
    if (instance == null) {
      instance = new LED();
    }
    return instance;
  }

  public void setLED(int[] rgb) {
    if (rgb != null) {
      clearAnimation();
      LED.setLEDs(rgb[0], rgb[1], rgb[2]);
    }
  }

  public void setLED(Animation animation, int animationSlot) {
    clearAnimation();
    LED.animate(animation, animationSlot);
  }

  public void setLEDMatrix(int[] rgb, int LEDStartIndex, int LEDLength) {
    clearAnimation();
    LED.setLEDs(rgb[0], rgb[1], rgb[2], 0, LEDStartIndex, LEDLength);
  }

  public void clearAnimation() {
    LED.clearAnimation(0);
    LED.clearAnimation(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}