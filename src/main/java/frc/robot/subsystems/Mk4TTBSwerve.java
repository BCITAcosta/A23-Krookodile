package frc.robot.subsystems;



import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.util.SwerveModuleConstants;

import frc.robot.Constants.SwerveDriveConstants;

public class Mk4TTBSwerve{
    // Turning Spark Information and Calls
    private final SparkMax m_turningSparkMax;
    private final SparkMaxConfig m_turningSparkMaxConfig;
    private final SparkAnalogSensor m_turningEncoder;
    private final SparkClosedLoopController m_turningController;

    // Driving Spark Information and Calls
    // private final SparkMax m_driveSparkMax;
    // private final SparkMaxConfig m_driveSparkMaxConfig;
    // private final RelativeEncoder m_driveEncoder;
    // private final SparkClosedLoopController m_drivePIDController;

    // Module Information and Calls
    private int moduleNum;

    private SwerveModuleState state;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
    private double m_angleOffset = 0.0;

    public Mk4TTBSwerve(int moduleNum, SwerveModuleConstants constants){
        this.moduleNum = moduleNum;

        m_turningSparkMaxConfig = new SparkMaxConfig();
        // m_driveSparkMaxConfig = new SparkMaxConfig();

        m_turningSparkMax = new SparkMax(constants.turnMotorID, MotorType.kBrushless);
        m_angleOffset = constants.angleOffset.getRadians();
        m_turningEncoder = m_turningSparkMax.getAnalog();
        m_turningController = m_turningSparkMax.getClosedLoopController();
        configTurningSpark();

        // m_driveSparkMax = new CANSparkMax(constants.driveMotorID, MotorType.kBrushless);
        // m_driveSparkMax.restoreFactoryDefaults();
        // m_driveSparkMax.setInverted(constants.inverted);
        // m_driveEncoder = m_driveSparkMax.getEncoder();
        // m_drivePIDController = m_driveSparkMax.getPIDController();
        // m_drivePIDController.setFeedbackDevice(m_driveEncoder);
        // m_driveEncoder.setPositionConversionFactor(DriveConstants.kDrivingEncoderPositionFactor);
        // m_driveEncoder.setVelocityConversionFactor(DriveConstants.kDrivingEncoderVelocityFactor);
        // m_drivePIDController.setP(0.04,0);
        // m_drivePIDController.setI(0,0);
        // m_drivePIDController.setD(0,0);
        // m_drivePIDController.setFF(1, 0);
        // m_drivePIDController.setOutputRange(-1, 1, 0);
        // m_driveSparkMax.setIdleMode(IdleMode.kBrake);
        // m_driveSparkMax.setSmartCurrentLimit(50);
        // m_driveSparkMax.setClosedLoopRampRate(0.05);
        // m_driveSparkMax.burnFlash();

        
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        // m_driveEncoder.setPosition(0);
    }

    public void setDesiredState(SwerveModuleState desiredState){
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(new Rotation2d(m_angleOffset));

        SwerveModuleState optimizedDesiredState = correctedDesiredState.optimize(Rotation2d(0.7));
        // SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, 
       // new Rotation2d(m_turningEncoder.getPosition()));

        m_turningController.setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);
        // m_drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        
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

    public void configTurningSpark(){
        m_turningSparkMaxConfig.idleMode(IdleMode.kBrake);
        m_turningSparkMaxConfig.inverted(true);
        m_turningSparkMaxConfig.smartCurrentLimit(40);
        m_turningSparkMaxConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAnalogSensor)
        .pidf(0.45,0.0,0.0,0.0)
        .outputRange(-1.0, 1.0)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 2*Math.PI);
        m_turningSparkMaxConfig.closedLoopRampRate(0.05);
        m_turningSparkMax.configure(m_turningSparkMaxConfig, null, null);
    }

    public void putSmartDashboard(){
        // SmartDashboard.putNumber(this.moduleNum + " Actual Angle", m_turningEncoder.getPosition());
        // SmartDashboard.putNumber(this.moduleNum + " Mod. Offset", m_angleOffset);
        // SmartDashboard.putNumber(this.moduleNum + " M Angle", m_turningEncoder.getPosition()-m_angleOffset);
        // SmartDashboard.putNumber(this.moduleNum + " Set Point", m_desiredState.angle.getDegrees());
        // SmartDashboard.putNumber(this.moduleNum + " Speed Setpoint", m_desiredState.speedMetersPerSecond);
        // SmartDashboard.putNumber(this.moduleNum + "Drive Encoder", m_driveEncoder.getPosition());
    }
}
