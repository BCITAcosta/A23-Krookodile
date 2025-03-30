package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    
    private static Elevator elevator;
    
    private static SparkMax leftLiftMotor, rightLiftMotor;
    private static SparkMaxConfig leftLiftConfig, rightLiftConfig;

    public Elevator(){
        configureMotors();
    }

    public static Elevator getInstance(){
        if (elevator == null){
            elevator = new Elevator();
        }
        return elevator;
    }

    private void configureMotors(){
        
        // Left Motor Configuration
        leftLiftConfig.inverted(false);
        leftLiftConfig.idleMode(IdleMode.kBrake);
        leftLiftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(ElevatorConstants.kP, 
        ElevatorConstants.kI,
        ElevatorConstants.kD,
        ElevatorConstants.kFF)
        .maxMotion.maxVelocity(1000) // Default Velocity is in Motor RPM
        .maxAcceleration(100) //Default Acceleration is in Motor RPM/s
        .allowedClosedLoopError(10);

        // Right Motor Configuration
        rightLiftConfig.apply(leftLiftConfig);
        rightLiftConfig.follow(ElevatorConstants.leftMotorID,true);
        
    }
}
