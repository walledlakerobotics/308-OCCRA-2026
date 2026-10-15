package frc.robot.constants

import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units

// meters/second
const val kSpeedFactor = 10

const val kFrontLeftMotorCANID = 2
const val kFrontRightMotorCANID = 4
const val kBackLeftMotorCANID = 8
const val kBackRightMotorCANID = 6

val kFrontLeftNeoTranslation = Translation2d(Units.inchesToMeters(10.0), Units.inchesToMeters(11.75))
val kFrontRightNeoTranslation = Translation2d(Units.inchesToMeters(10.0), Units.inchesToMeters(-11.75))
val kBackLeftNeoTranslation = Translation2d(Units.inchesToMeters(-10.0), Units.inchesToMeters(11.75))
val kBackRightNeoTranslation = Translation2d(Units.inchesToMeters(-10.0), Units.inchesToMeters(-11.75))


val kSparkMaxConfig: SparkMaxConfig
    get() {
        val config = SparkMaxConfig()

        // pid control
        config.closedLoop
            .pid(0.25, 0.0, 0.1)
            .maxOutput(1.0) // the max duty cycle coming from PID

        config
            .idleMode(SparkBaseConfig.IdleMode.kCoast)
            .smartCurrentLimit(45) // amps

        val c = Math.PI * Units.inchesToMeters(6.0)

        config.encoder
            .positionConversionFactor(c / 8.45865)
            .velocityConversionFactor(c / 8.45865 / 60.0)

        return config
    }

val kLeftConfig: SparkMaxConfig
    get() {
        val config = kSparkMaxConfig
        config.inverted(false)

        return config
    }

val kRightConfig: SparkMaxConfig
    get() {
        val config = kSparkMaxConfig
        config.inverted(true)

        return config
    }
    