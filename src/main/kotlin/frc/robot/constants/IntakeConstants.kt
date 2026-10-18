package frc.robot.constants

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
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
    get() = TalonFXConfiguration()
        .withMotorOutput(
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
        )
        .withCurrentLimits(
            CurrentLimitsConfigs()
                .withStatorCurrentLimit(100.0)
                .withSupplyCurrentLimit(60.0)
        )
        .withSlot0(
            Slot0Configs()
                .withKP(0.4)
                .withKI(0.0)
                .withKD(0.1)
        )