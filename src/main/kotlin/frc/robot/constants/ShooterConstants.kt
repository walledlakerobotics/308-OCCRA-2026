package frc.robot.constants

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.NeutralModeValue

const val kShootCANID = 24

// rotatons/s 
const val kShootingVelocity = 10.0

val kShooterConfig: TalonFXConfiguration 
    get() {
        val config = TalonFXConfiguration()
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast
        
        val gainConfig = Slot0Configs()
        
        gainConfig.kP = 0.5
        gainConfig.kI = 0.0
        gainConfig.kD = 0.0
        
        config.withSlot0(gainConfig)
        
        return config
    }