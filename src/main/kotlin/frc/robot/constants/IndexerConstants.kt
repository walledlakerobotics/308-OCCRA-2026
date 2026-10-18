package frc.robot.constants

import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import kotlin.math.PI

const val kLeftIndexerCANID = 21
const val kRightIndexerCANID = 23

const val kLeftSensorChannel = 1
const val kRightSensorChannel = 0

private val kIndexerMotorConfig: SparkBaseConfig
    get() {
        val config = SparkMaxConfig()

        config
            .idleMode(SparkBaseConfig.IdleMode.kCoast)
            .smartCurrentLimit(45) // amps
            .voltageCompensation(12.0) // volts

        return config
    }
    
val kLeftIndexerMotorConfig: SparkBaseConfig
    get() = kIndexerMotorConfig.inverted(false)

val kRightIndexerMotorConfig: SparkBaseConfig
    get() = kIndexerMotorConfig.inverted(false)