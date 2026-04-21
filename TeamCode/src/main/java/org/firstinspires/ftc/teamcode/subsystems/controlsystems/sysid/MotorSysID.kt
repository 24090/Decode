package org.firstinspires.ftc.teamcode.subsystems.controlsystems.sysid

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.VoltageCompensatedMotor
import kotlin.math.abs

public open class MotorSysID(name: String, getMotor: HardwareMap.() -> VoltageCompensatedMotor) : SysIDRoutine<VoltageCompensatedMotor>(
    name,
    arrayOf("position"),
    {
       hardwareMap.getMotor()
    },
    { signal ->
        power = signal
        arrayOf(currentPosition)
    },
    { signal, t -> signal > 1.0 },
)