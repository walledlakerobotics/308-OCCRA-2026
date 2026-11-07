package frc.robot.constants

import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Millimeters
import frc.robot.subsystems.drivetrain.EncoderDirection
import kotlin.math.PI

// meters/second
const val kTranslationFactor = 3
const val kRotationFactor = 5

const val kFrontLeftMotorCANID = 2
const val kFrontRightMotorCANID = 4
const val kBackLeftMotorCANID = 8
const val kBackRightMotorCANID = 6

val kFrontLeftNeoTranslation = Translation2d(Units.inchesToMeters(10.0), Units.inchesToMeters(11.75))
val kFrontRightNeoTranslation = Translation2d(Units.inchesToMeters(10.0), Units.inchesToMeters(-11.75))
val kBackLeftNeoTranslation = Translation2d(Units.inchesToMeters(-10.0), Units.inchesToMeters(11.75))
val kBackRightNeoTranslation = Translation2d(Units.inchesToMeters(-10.0), Units.inchesToMeters(-11.75))

private val kWheelCircumference = PI * Units.inchesToMeters(6.0)
private const val kGearReduction = 8.45865

private val kRotationsToMeters = kWheelCircumference / kGearReduction

private val kSparkMaxConfig: SparkBaseConfig
    get() {
        val config = SparkMaxConfig()

        // pid control
        config.closedLoop
            .pid(0.1, 0.0, 0.0)
            .maxOutput(1.0) // the max duty cycle coming from PID
            .feedForward
            .kV(12.0 / (5676.0 * kRotationsToMeters / 60.0))

        config
            .idleMode(SparkBaseConfig.IdleMode.kCoast)
            .smartCurrentLimit(45) // amps
            .voltageCompensation(12.0) // volts

        config.encoder
            .positionConversionFactor(kRotationsToMeters)
            .velocityConversionFactor(kRotationsToMeters / 60.0)

        return config
    }

val kLeftConfig: SparkBaseConfig
    get() = kSparkMaxConfig.inverted(false)

val kRightConfig: SparkBaseConfig
    get() = kSparkMaxConfig.inverted(true)
    
// Odometry pods
const val kPodWheelDiameter = 38 // mm
const val kPodWheelCircumference = kPodWheelDiameter * Math.PI // mm

const val kTicksPerRevolution = 4096

const val kTicksPerMM = kTicksPerRevolution / kPodWheelCircumference

val kOffsetX = Millimeters.of(275.0)!!
val kOffsetY = Millimeters.of(68.0)!!

val kXEncoderDirection = EncoderDirection.BACKWARD
val kYEncoderDirection = EncoderDirection.FORWARD