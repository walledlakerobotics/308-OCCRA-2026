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
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.MecanumDriveKinematics
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.constants.*
import frc.robot.lib.BLine.FollowPath
import frc.robot.lib.BLine.Path
import pinpointrio.Pinpoint
import kotlin.math.PI
import kotlin.math.absoluteValue

@Logged
class Drivetrain : SubsystemBase(), AutoCloseable {
    private val frontLeftNeo = SparkMax(kFrontLeftMotorCANID, SparkLowLevel.MotorType.kBrushless)
    private val frontRightNeo = SparkMax(kFrontRightMotorCANID, SparkLowLevel.MotorType.kBrushless)
    private val backLeftNeo = SparkMax(kBackLeftMotorCANID, SparkLowLevel.MotorType.kBrushless)
    private val backRightNeo = SparkMax(kBackRightMotorCANID, SparkLowLevel.MotorType.kBrushless)

    private val odometry = Pinpoint(SerialPort.Port.kUSB)

    private val distanceSensor = AnalogInput(0)

    private var odometryOutOfDate = false

    var position
        get() = odometry.position
        set(value) {
            odometryOutOfDate = true
            odometry.position = value
        }

    var heading
        get() = odometry.rotation2d
        set(value) {
            odometryOutOfDate = true
            odometry.rotation2d = value
        }

    val velocity
        get() = odometry.velocity

    val distanceToWall
        get() = distanceSensor.value

    val builder = FollowPath.Builder(
        this,
        { position },
        { velocity },
        { drive(it) },
        PIDController(3.0, 0.0, 1.0),
        PIDController(5.0, 0.0, 0.0),
        PIDController(7.0, 0.0, 0.0)
    )

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

    private val fieldWidget = Field2d()

    init {
        frontLeftNeo.configure(kLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        frontRightNeo.configure(kRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        backLeftNeo.configure(kLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        backRightNeo.configure(kRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        odometry.setEncoderOffsets(kOffsetX, kOffsetY)
        odometry.setEncoderDirections(kXEncoderDirection, kYEncoderDirection)

        odometry.setEncoderResolution(kTicksPerMM)

        SmartDashboard.putData("Drivetrain/Field", fieldWidget)

        RobotModeTriggers.disabled()
            .onTrue(Commands.runOnce({ setIdleMode(SparkBaseConfig.IdleMode.kCoast) }))
            .onFalse(Commands.runOnce({ setIdleMode(SparkBaseConfig.IdleMode.kBrake) }))

        SmartDashboard.putData("Odometry/Reset", runOnce { position = Pose2d.kZero }.ignoringDisable(true))
    }

    fun followPath(path: Path, resetOdometry: Boolean = false): Command {
        if (resetOdometry) {
            builder.withPoseReset { position = it }
        } else {
            builder.withPoseReset { /* do nothing */ }
        }

        return builder.build(path)
    }

    fun drive(speeds: ChassisSpeeds, fieldRelative: Boolean = false) {
        var speeds = speeds

        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, heading)
        }

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
    }

    fun drive(x: () -> Double, y: () -> Double, theta: () -> Double, fieldRelative: Boolean = true): Command {
        val holdHeadingController = PIDController(5.0, 0.0, 0.0).apply {
            enableContinuousInput(-PI, PI)
        }

        val holdXController = PIDController(5.0, 0.0, 0.0)
        val holdYController = PIDController(5.0, 0.0, 0.0)

        var heldHeading: Rotation2d? = null
        var heldTranslation: Translation2d? = null

        val xLimiter = SlewRateLimiter(6.0, Double.NEGATIVE_INFINITY, 0.0)
        val yLimiter = SlewRateLimiter(6.0, Double.NEGATIVE_INFINITY, 0.0)

        return startRun({
            heldHeading = null
            heldTranslation = null

            val velocity = if (fieldRelative)
                ChassisSpeeds.fromRobotRelativeSpeeds(velocity, heading) else velocity

            xLimiter.reset(velocity.vxMetersPerSecond)
            yLimiter.reset(velocity.vyMetersPerSecond)
        }) {
            var fieldRelative = fieldRelative

            val x = xLimiter.calculate(x() * kTranslationFactor)
            val y = yLimiter.calculate(y() * kTranslationFactor)
            val theta = theta() * kRotationFactor

            val speeds = ChassisSpeeds(x, y, theta)

            if (odometryOutOfDate) {
                heldHeading = null
                heldTranslation = null

                odometryOutOfDate = false
            }

            if (heldHeading == null) {
                if (
                    velocity.omegaRadiansPerSecond.absoluteValue < 0.01 &&
                    speeds.omegaRadiansPerSecond.absoluteValue < 0.01
                ) {
                    holdHeadingController.reset()
                    heldHeading = position.rotation
                }
            } else {
                if (speeds.omegaRadiansPerSecond.absoluteValue < 0.01) {
                    speeds.omegaRadiansPerSecond =
                        holdHeadingController.calculate(
                            position.rotation.radians, heldHeading!!.radians) +
                        speeds.vyMetersPerSecond * 0.1
                } else {
                    heldHeading = null
                }
            }

            if (heldTranslation == null) {
                if (
                    velocity.vxMetersPerSecond.absoluteValue < 0.01 &&
                    velocity.vyMetersPerSecond.absoluteValue < 0.01 &&
                    speeds.vxMetersPerSecond.absoluteValue < 0.01 &&
                    speeds.vyMetersPerSecond.absoluteValue < 0.01
                ) {
                    holdXController.reset()
                    holdYController.reset()
                    heldTranslation = position.translation
                }
            } else {
                if (
                    speeds.vxMetersPerSecond.absoluteValue < 0.01 &&
                    speeds.vyMetersPerSecond.absoluteValue < 0.01
                ) {
                    fieldRelative = true

                    speeds.vxMetersPerSecond = holdXController.calculate(position.x, heldTranslation!!.x)
                    speeds.vyMetersPerSecond = holdYController.calculate(position.y, heldTranslation!!.y)
                } else {
                    heldTranslation = null
                }
            }

            drive(speeds, fieldRelative)
        }.finallyDo { -> stop() }
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
        distanceSensor.close()
    }

    private val sysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism({ volts ->
            frontLeftNeo.setVoltage(volts)
            frontRightNeo.setVoltage(volts)
            backLeftNeo.setVoltage(volts)
            backRightNeo.setVoltage(volts)
        }, null, this, "shooter")
    )

    val sysIdForwardQuasistatic: Command
        get() = sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)

    val sysIdReverseQuasistatic: Command
        get() = sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)

    val sysIdForwardDynamic: Command
        get() = sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)

    val sysIdReverseDynamic: Command
        get() = sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)

    override fun periodic() {
        fieldWidget.robotPose = position
    }
}