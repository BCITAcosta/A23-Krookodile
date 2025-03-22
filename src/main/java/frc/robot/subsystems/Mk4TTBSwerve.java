package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.util.SwerveModuleConstants;

import frc.robot.Constants.SwerveDriveConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Mk4TTBSwerve{
    // Turning Spark Information and Calls
    private final SparkMax m_turningSparkMax;
    private final SparkMaxConfig m_turningSparkMaxConfig;
    private final SparkAnalogSensor m_turningEncoder;
    private final SparkClosedLoopController m_turningController;

    // Driving Spark Information and Calls
    private final SparkMax m_driveSparkMax;
    private final SparkMaxConfig m_driveSparkMaxConfig;
    private final RelativeEncoder m_driveEncoder;
    private final SparkClosedLoopController m_driveController;

    // Module Information and Calls
    private int moduleNum;
    private SwerveModuleConstants m_constants;

    private SwerveModuleState state;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
    private double m_angleOffset = 0.0;

    public Mk4TTBSwerve(int moduleNum, SwerveModuleConstants constants){
        this.moduleNum = moduleNum;

        m_constants = constants;

        m_turningSparkMaxConfig = new SparkMaxConfig();
        m_driveSparkMaxConfig = new SparkMaxConfig();

        m_turningSparkMax = new SparkMax(constants.turnMotorID, MotorType.kBrushless);
        m_angleOffset = constants.angleOffset.getRadians();
        m_turningEncoder = m_turningSparkMax.getAnalog();
        m_turningController = m_turningSparkMax.getClosedLoopController();
        configTurningSpark();

        m_driveSparkMax = new SparkMax(constants.driveMotorID, MotorType.kBrushless);
        m_driveEncoder = m_driveSparkMax.getEncoder();
        m_driveController = m_driveSparkMax.getClosedLoopController();
        configDriveSpark();
        
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_driveEncoder.setPosition(0);
    }

    private void configTurningSpark(){
        m_turningSparkMaxConfig.idleMode(IdleMode.kCoast);
        m_turningSparkMaxConfig.inverted(false);
        m_turningSparkMaxConfig.smartCurrentLimit(40);
        m_turningSparkMaxConfig.analogSensor.inverted(true);
        m_turningSparkMaxConfig.analogSensor.positionConversionFactor((2*Math.PI)/3.3);
        m_turningSparkMaxConfig.analogSensor.velocityConversionFactor(((2*Math.PI)/3.3)/60);
        m_turningSparkMaxConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAnalogSensor)
        .pidf(0.45,0.0,0.0,0.0)
        .outputRange(-1.0, 1.0)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 2*Math.PI);
        m_turningSparkMaxConfig.closedLoopRampRate(0.05);
        m_turningSparkMax.configure(m_turningSparkMaxConfig, null, null);
    }

    private void configDriveSpark(){
        m_driveSparkMaxConfig.idleMode(IdleMode.kBrake);
        m_driveSparkMaxConfig.inverted(m_constants.inverted);
        m_driveSparkMaxConfig.smartCurrentLimit(50);
        m_driveSparkMaxConfig.encoder.positionConversionFactor(SwerveDriveConstants.kDrivingEncoderPositionFactor);
        m_driveSparkMaxConfig.encoder.velocityConversionFactor(SwerveDriveConstants.kDrivingEncoderVelocityFactor);
        m_driveSparkMaxConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(0.04,0.0,0.0,1.0)
        .outputRange(-1.0, 1.0);
        m_driveSparkMaxConfig.closedLoopRampRate(0.05);
        m_driveSparkMax.configure(m_driveSparkMaxConfig,null,null);

    }


    public void setDesiredState(SwerveModuleState desiredState){
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(new Rotation2d(m_angleOffset));
        correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

        m_turningController.setReference(correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);
        m_driveController.setReference(correctedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
        
        m_desiredState = desiredState;
    }



    public SwerveModuleState getDesiredState(){
        return state;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()-m_angleOffset));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition()-m_angleOffset));
    }

    public void stop(){
        m_turningSparkMax.set(0.0);
    }

    public int getModuleNumber(){
        return this.moduleNum;
    }




    public void putSmartDashboard(){
        SmartDashboard.putNumber(this.moduleNum + " Actual Angle", m_turningEncoder.getPosition());
        SmartDashboard.putNumber(this.moduleNum + " Mod. Offset", m_angleOffset);
        SmartDashboard.putNumber(this.moduleNum + " M Angle", m_turningEncoder.getPosition()-m_angleOffset);
        SmartDashboard.putNumber(this.moduleNum + " Set Point", m_desiredState.angle.getDegrees());
        SmartDashboard.putNumber(this.moduleNum + " Speed Setpoint", m_desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber(this.moduleNum + "Drive Encoder", m_driveEncoder.getPosition());
    }
}
