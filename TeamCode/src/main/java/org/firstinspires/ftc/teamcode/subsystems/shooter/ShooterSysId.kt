package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.sysid.SysIDRoutine
import kotlin.math.abs

@TeleOp(name = "Shooter SysID")
class ShooterSysId : SysIDRoutine<Shooter>(
    "Shooter",
    listOf("positionLeft", "positionRight"),
    {
        Shooter(hardwareMap)
    },
    { signal ->
        motorLeft.power = signal
        motorRight.power = signal
        listOf(motorLeft.currentPosition, motorRight.currentPosition)
    },
    {
        abs(motorLeft.power/motorLeft.compensationFactor) >= 1.0
    },
)