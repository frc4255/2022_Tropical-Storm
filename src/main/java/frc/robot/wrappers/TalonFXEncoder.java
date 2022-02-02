// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * 
 * A wrapper class used to access the TalonFX built in encoders in the same way as the Encoder class.
 * 
*/
public class TalonFXEncoder {

    WPI_TalonFX motor;
    WPI_TalonFX[] motors;

    boolean multiple = false;

    double distancePerPulse = 1.0;


    public TalonFXEncoder(WPI_TalonFX source){
        motor = source;

        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }
    

    public TalonFXEncoder(WPI_TalonFX ...sources){
        motors = sources;

        for(WPI_TalonFX motor : motors){

            motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        }

        multiple = true;
    }


    private double getRawPos(){

        double rawCounts = 0.0;

        if(multiple){

            for(WPI_TalonFX motor : motors){

                double motorCounts = motor.getSelectedSensorPosition();

                if(rawCounts == 0.0){

                    rawCounts += motorCounts;

                } else{

                    if(Math.signum(rawCounts) != Math.signum(motorCounts)){

                        rawCounts -= motorCounts;

                    } else{

                        rawCounts += motorCounts;

                    }
                }
            }

            rawCounts /= (double) motors.length;
            
        } else{

            rawCounts = motor.getSelectedSensorPosition();

        }

        return rawCounts;
    }


    private double getRawVel(){

        double rawVel = 0.0;

        if(multiple){

            for(WPI_TalonFX motor : motors){

                double motorVel = motor.getSelectedSensorVelocity();

                if(rawVel == 0.0){

                    rawVel += motorVel;

                } else{

                    if(Math.signum(rawVel) != Math.signum(motorVel)){

                        rawVel -= motorVel;

                    } else{

                        rawVel += motorVel;

                    }
                }
            }

            rawVel /= (double) motors.length;
            
        } else{

            rawVel = motor.getSelectedSensorVelocity();

        }

        return rawVel * 10.0;
    }


    /**
     * Sets the distance per pulse for the encoder, which is 1 by default.
     * Distance per pulse affects value returned by getDistance and getVelocity
     * @param factor The distance per pulse for the encoder
     */
    public void setDistancePerPulse(double factor){
        distancePerPulse = factor;
    }

    
    /**
     * 
     * @return Distance traveled in distance units
     */
    public double getDistance(){
        return distancePerPulse * getRawPos();
    }


    /**
     * 
     * @return Rotational velocity in distance units / second
     */
    public double getVelocity(){
        return distancePerPulse * getRawVel();
    }


    /**
     * 
     * @return Rotations per minute
     */
    public double getRPM(){

        double factor = 60.0 / 2048.0;

        return factor * getRawVel();
    }


    /**
     * Reset the encoders to 0.0
     */
    public void reset(){

        if(multiple){

            for(WPI_TalonFX motor : motors){

                motor.setSelectedSensorPosition(0.0);

            }
        
        } else{

            motor.setSelectedSensorPosition(0.0);

        }
    }
}
