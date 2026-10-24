package frc.robot.constants

import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig

const val kFeederCANID = 25

val kFeederMotorConfig: SparkBaseConfig
    get() {
        val config = SparkMaxConfig()

        config
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(20) // amps
            .voltageCompensation(12.0) // volts
            .inverted(true)

        return config
    }