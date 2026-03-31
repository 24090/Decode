package org.firstinspires.ftc.teamcode.subsystems.controlsystems.sysid

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.VoltageCompensatedMotor
import kotlin.math.abs


class MotorSysID(name: String, getMotor: HardwareMap.() -> VoltageCompensatedMotor) : SysIDRoutine<VoltageCompensatedMotor>(
    name,
    listOf("position"),
    {
        hardwareMap.getMotor()
    },
    { signal ->
        power = signal
        listOf(currentPosition)
    },
    {
        abs(power/compensationFactor) > 1.0
    },
)