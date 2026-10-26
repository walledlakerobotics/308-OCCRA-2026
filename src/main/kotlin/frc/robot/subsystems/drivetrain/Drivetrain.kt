package frc.robot.subsystems.drivetrain

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
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
import frc.robot.lib.BLine.FollowPath
import kotlin.math.absoluteValue

@Logged
class Drivetrain : SubsystemBase(), AutoCloseable {
    private val frontLeftNeo = SparkMax(kFrontLeftMotorCANID, SparkLowLevel.MotorType.kBrushless)
    private val frontRightNeo = SparkMax(kFrontRightMotorCANID, SparkLowLevel.MotorType.kBrushless)
    private val backLeftNeo = SparkMax(kBackLeftMotorCANID, SparkLowLevel.MotorType.kBrushless)
    private val backRightNeo = SparkMax(kBackRightMotorCANID, SparkLowLevel.MotorType.kBrushless)

    private val odometry = Pinpoint(SerialPort.Port.kUSB)

    val position
        get() = odometry.position

    val heading
        get() = odometry.position.rotation!!

    val velocity
        get() = odometry.velocity

    val builder = FollowPath.Builder(
        this,
        { position },
        { velocity },
        { drive(it) },
        PIDController(0.5, 0.0, 0.0),
        PIDController(0.5, 0.0, 0.0),
        PIDController(0.5, 0.0, 0.0)
    ).withPoseReset { odometry.position = it }!!

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

        odometry.setEncoderOffsets(kOffsetX, kOffsetY)
        odometry.setEncoderDirections(kXEncoderDirection, kYEncoderDirection)
        
        odometry.setEncoderResolution(kTicksPerMM)

        RobotModeTriggers.disabled()
            .onTrue(Commands.runOnce({ setIdleMode(SparkBaseConfig.IdleMode.kCoast) }))
            .onFalse(Commands.runOnce({ setIdleMode(SparkBaseConfig.IdleMode.kBrake) }))
    }

    fun drive(speeds: ChassisSpeeds) {
        // gets kinematic velocity's
        val wheelSpeeds = kinematics.toWheelSpeeds(speeds)

        // sets the set point.
        frontLeftNeo.closedLoopController.setSetpoint(
            wheelSpeeds.frontLeftMetersPerSecond,
            SparkBase.ControlType.kVelocity
        )
        frontRightNeo.closedLoopController.setSetpoint(
            wheelSpeeds.frontRightMetersPerSecond,
            SparkBase.ControlType.kVelocity
        )
        backRightNeo.closedLoopController.setSetpoint(
            wheelSpeeds.rearRightMetersPerSecond,
            SparkBase.ControlType.kVelocity
        )
        backLeftNeo.closedLoopController.setSetpoint(
            wheelSpeeds.rearLeftMetersPerSecond,
            SparkBase.ControlType.kVelocity
        )

        SmartDashboard.putNumber("Drive/Goal/FrontLeft", wheelSpeeds.frontLeftMetersPerSecond)
        SmartDashboard.putNumber("Drive/Goal/FrontRight", wheelSpeeds.frontRightMetersPerSecond)
        SmartDashboard.putNumber("Drive/Goal/RearLeft", wheelSpeeds.rearLeftMetersPerSecond)
        SmartDashboard.putNumber("Drive/Goal/RearRight", wheelSpeeds.rearRightMetersPerSecond)
    }

    fun drive(x: () -> Double, y: () -> Double, theta: () -> Double): Command {
        val driftPID = PIDController(1.0, 0.0, 0.0)
        driftPID.enableContinuousInput(-180.0, 180.0)

        var prevHeading: Rotation2d? = null

        return startRun({ driftPID.reset() }) {
            val x = x() * kTranslationFactor
            val y = y() * kTranslationFactor
            val theta = theta() * kRotationFactor

            val speeds = ChassisSpeeds(x, y, theta)

            if (speeds.omegaRadiansPerSecond.absoluteValue < 0.05) {
                prevHeading = prevHeading ?: odometry.position.rotation

                speeds.omegaRadiansPerSecond =
                    driftPID.calculate(odometry.position.rotation.degrees, prevHeading!!.degrees) +
                            speeds.vyMetersPerSecond * 0.1
            } else {
                prevHeading = null
            }

            drive(speeds)
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

        odometry.close()
    }
}