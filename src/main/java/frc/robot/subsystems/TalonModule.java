package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.hardware.TalonFX;

public class TalonModule extends TalonFX{

  public TalonModule(int deviceId) {
    super(deviceId, "rio");
  }
}
