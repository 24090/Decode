package org.firstinspires.ftc.teamcode.subsystems.drive.tuners

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmodes.commands.Robot
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.sysid.SysIDRoutine
import kotlin.math.abs

@TeleOp(name = "Drive SysID")
class DriveSysID() : SysIDRoutine<Robot>(
    "Drive",
    listOf("positionLeft", "positionRight"),
    { Robot(hardwareMap, telemetry) },
    { signal ->
        drive.localizer.pinpoint.update()

        drive.flMotor.power = signal
        drive.frMotor.power = signal
        drive.blMotor.power = signal
        drive.brMotor.power = signal

        listOf(drive.localizer.x, drive.localizer.y, drive.localizer.heading)
    },
    {
        abs(drive.flMotor.power/drive.flMotor.compensationFactor) >= 1.0
    },
)