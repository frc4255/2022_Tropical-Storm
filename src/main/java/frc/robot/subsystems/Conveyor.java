// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Shuphlebord;

public class Conveyor extends SubsystemBase {

    
  public static enum STATES {INDEX, FEED, STOP};
  public static enum SUBSTATES {IDLE, MISFIRE, VOMIT, STORE, LOAD}
  
  public static STATES State = STATES.INDEX;
  public static SUBSTATES Substate = SUBSTATES.IDLE;

  public WPI_TalonFX motor;
  public double liftSpeed = -0.26;
  public double lowerSpeed = 0.26;

  private final I2C.Port i2cPort = I2C.Port.kMXP;

  private static ColorSensorV3 colorSensor;

  static AnalogInput IRSensor = new AnalogInput(0);

  public static int ballsInConveyor = 0;

  /** Creates a new Conveyor. */
  public Conveyor() {

    motor = new WPI_TalonFX(Constants.Conveyor.motor);

    colorSensor = new ColorSensorV3(i2cPort);
    IRSensor.setAverageBits(4);

  }


  public void set(double speed) {

   this.motor.set(speed); 
  
  }


  public void stop() {

    this.set(0.0);

  }

  
  /**
   * @return True if a ball is at the second position, false otherwise
   */
  public static boolean hasSecondBall(){

    double value = IRSensor.getAverageValue();

    if(value < 1000.0){

      return true;

    }

    return false;

  }


  /**
   * Returns an int: 0 if ball is correct color, 1 if incorrect, 2 if no ball
   * 
   * @return Number code showing state of ball at hopper
   */
  public static int hasCorrectBall(){

    int code = 2;
    Color color = colorSensor.getColor();
    int proximity = colorSensor.getProximity();
    int proximityThreshold = 80;
    double blueThreshold = 0.3;
    double redThreshold = 0.4;

    if(proximity > proximityThreshold){
 
      if(color.red > redThreshold){

        code = Constants.allianceColor == 0 ? 0 : 1;

      } else if(color.blue > blueThreshold){

        code = Constants.allianceColor == 0 ? 1 : 0;

      }

      return code;

    }

    return 2;
  }
  

  public void displayConveyorValues(){

    Color color = colorSensor.getColor();
    int proximity = colorSensor.getProximity();
    int secondBall = hasSecondBall() == true ? 0 : 1;

    Shuphlebord.conveyorData.updateEntry("Red", color.red);
    Shuphlebord.conveyorData.updateEntry("Blue", color.blue);
    Shuphlebord.conveyorData.updateEntry("Green", color.green);
    Shuphlebord.conveyorData.updateEntry("Proximity", proximity);
    Shuphlebord.conveyorData.updateEntry("Correct Ball", hasCorrectBall());
    Shuphlebord.conveyorData.updateEntry("Second Ball", secondBall);
    Shuphlebord.conveyorData.updateEntry("Number of Balls", ballsInConveyor);
    //System.out.print("SUBSTATE: " + Substate.toString() + ", BALLS: " + ballsInConveyor);
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
