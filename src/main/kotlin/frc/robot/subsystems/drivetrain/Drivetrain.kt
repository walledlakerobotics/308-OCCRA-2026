package frc.robot.subsystems.drivetrain

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.MecanumDriveKinematics
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import frc.robot.constants.*

@Logged
class Drivetrain : SubsystemBase(), AutoCloseable {
    private val frontLeftNeo = SparkMax(kFrontLeftMotorCANID, SparkLowLevel.MotorType.kBrushless)
    private val frontRightNeo = SparkMax(kFrontRightMotorCANID, SparkLowLevel.MotorType.kBrushless)
    private val backLeftNeo = SparkMax(kBackLeftMotorCANID, SparkLowLevel.MotorType.kBrushless)
    private val backRightNeo = SparkMax(kBackRightMotorCANID, SparkLowLevel.MotorType.kBrushless)
    
    private val driveOdometry = DriveOdometry(SerialPort.Port.kUSB1)

    private val kinematics = MecanumDriveKinematics(
        kFrontLeftNeoTranslation,
        kFrontRightNeoTranslation,
        kBackLeftNeoTranslation,
        kBackRightNeoTranslation
    )
    
    val wheelSpeeds
        get() = MecanumDriveWheelSpeeds(
            frontLeftNeo.encoder.velocity,
            frontRightNeo.encoder.velocity,
            backLeftNeo.encoder.velocity,
            backRightNeo.encoder.velocity,
        )

    val wheelCurrents
        get() = MecanumDriveWheelSpeeds(
            frontLeftNeo.outputCurrent,
            frontRightNeo.outputCurrent,
            backLeftNeo.outputCurrent,
            backRightNeo.outputCurrent,
        )

    init {
        frontLeftNeo.configure(kLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        frontRightNeo.configure(kRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        backLeftNeo.configure(kLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        backRightNeo.configure(kRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        RobotModeTriggers.disabled()
            .onTrue(Commands.runOnce({ setIdleMode(SparkBaseConfig.IdleMode.kCoast) }))
            .onFalse(Commands.runOnce({ setIdleMode(SparkBaseConfig.IdleMode.kBrake) }))
    }
    
    fun drive(speeds: ChassisSpeeds) {
        // gets kinematic velocity's
        val v = kinematics.toWheelSpeeds(speeds)

        // sets the set point.
        frontLeftNeo.closedLoopController.setSetpoint(v.frontLeftMetersPerSecond, SparkBase.ControlType.kVelocity)
        frontRightNeo.closedLoopController.setSetpoint(v.frontRightMetersPerSecond, SparkBase.ControlType.kVelocity)
        backRightNeo.closedLoopController.setSetpoint(v.rearRightMetersPerSecond, SparkBase.ControlType.kVelocity)
        backLeftNeo.closedLoopController.setSetpoint(v.rearLeftMetersPerSecond, SparkBase.ControlType.kVelocity)

        SmartDashboard.putNumber("Drive/Goal/FrontLeft", v.frontLeftMetersPerSecond)
        SmartDashboard.putNumber("Drive/Goal/FrontRight", v.frontRightMetersPerSecond)
        SmartDashboard.putNumber("Drive/Goal/RearLeft", v.rearLeftMetersPerSecond)
        SmartDashboard.putNumber("Drive/Goal/RearRight", v.rearRightMetersPerSecond)
    }

    fun drive(x: () -> Double, y: () -> Double, theta: () -> Double): Command {
        return run {
            drive(
                ChassisSpeeds(
                    x() * kTranslationFactor,
                    y() * kTranslationFactor,
                    theta() * kRotationFactor
                )
            )
        }
    }

    fun stop() {
        drive(ChassisSpeeds())
    }

    fun setIdleMode(mode: SparkBaseConfig.IdleMode) {
        val config = SparkMaxConfig().idleMode(mode)
        
        frontLeftNeo.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters)
        frontRightNeo.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters)
        backLeftNeo.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters)
        backRightNeo.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters)
    }
    
    override fun close() {
        frontLeftNeo.close()
        frontRightNeo.close()
        backLeftNeo.close()
        backRightNeo.close()

        driveOdometry.close()
    }
}