package frc.robot.constants

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.NeutralModeValue

const val kShootCANID = 24

// rotatons/s 
const val kShootingVelocity = 10.0

val kShooterConfig: TalonFXConfiguration 
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
                .withKP(0.1)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.18)
                .withKV(0.11418)
                .withKA(0.0)
        )