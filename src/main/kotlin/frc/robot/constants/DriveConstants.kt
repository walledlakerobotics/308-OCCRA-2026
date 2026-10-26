package frc.robot.constants

import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Millimeters
import frc.robot.subsystems.drivetrain.EncoderDirection

// meters/second
const val kTranslationFactor = 5
const val kRotationFactor = 7

const val kFrontLeftMotorCANID = 2
const val kFrontRightMotorCANID = 4
const val kBackLeftMotorCANID = 8
const val kBackRightMotorCANID = 6

val kFrontLeftNeoTranslation = Translation2d(Units.inchesToMeters(10.0), Units.inchesToMeters(11.75))
val kFrontRightNeoTranslation = Translation2d(Units.inchesToMeters(10.0), Units.inchesToMeters(-11.75))
val kBackLeftNeoTranslation = Translation2d(Units.inchesToMeters(-10.0), Units.inchesToMeters(11.75))
val kBackRightNeoTranslation = Translation2d(Units.inchesToMeters(-10.0), Units.inchesToMeters(-11.75))


private val kSparkMaxConfig: SparkBaseConfig
    get() {
        val config = SparkMaxConfig()

        // pid control
        config.closedLoop
            .pid(0.25, 0.0, 0.1)
            .maxOutput(1.0) // the max duty cycle coming from PID

        config
            .idleMode(SparkBaseConfig.IdleMode.kCoast)
            .smartCurrentLimit(45) // amps
            .voltageCompensation(12.0) // volts

        val c = Math.PI * Units.inchesToMeters(6.0)

        config.encoder
            .positionConversionFactor(c / 8.45865)
            .velocityConversionFactor(c / 8.45865 / 60.0)

        return config
    }

val kLeftConfig: SparkBaseConfig
    get() = kSparkMaxConfig.inverted(false)

val kRightConfig: SparkBaseConfig
    get() = kSparkMaxConfig.inverted(true)
    
// Odometry pods
const val kWheelDiameter = 38 // mm
const val kWheelCircumference = kWheelDiameter * Math.PI // mm

const val kTicksPerRevolution = 4096

const val kTicksPerMM = kTicksPerRevolution / kWheelCircumference

val kOffsetX = Millimeters.of(0.0)!!
val kOffsetY = Millimeters.of(0.0)!!

val kXEncoderDirection = EncoderDirection.BACKWARD
val kYEncoderDirection = EncoderDirection.FORWARD