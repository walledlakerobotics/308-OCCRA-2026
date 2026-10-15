package frc.robot.constants

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Millimeters
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.measure.Distance
import kotlin.math.PI

const val kIntakeCANID = 22

// rotatons/s
val kIntakeingVelocity = RotationsPerSecond.of(3.0)!!

val kIntakeerConfig: TalonFXConfiguration
    get() {
        val config = TalonFXConfiguration()

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast

        val gainConfig = Slot0Configs()

        gainConfig.kP = 0.4
        gainConfig.kI = 0.0
        gainConfig.kD = 0.1

        config.withSlot0(gainConfig)

        return config
    }