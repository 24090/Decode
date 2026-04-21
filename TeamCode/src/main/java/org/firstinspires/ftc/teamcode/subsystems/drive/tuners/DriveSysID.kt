package org.firstinspires.ftc.teamcode.subsystems.drive.tuners

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmodes.commands.Robot
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.sysid.SysIDRoutine
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import kotlin.math.abs

@TeleOp(name = "Drive SysID")
class DriveSysID() : SysIDRoutine<Robot>(
    "Drive",
    arrayOf("x", "y", "heading"),
    {
        val robot = Robot(hardwareMap, telemetry)
        robot.drive.localizer.pose = Pose(0.0, 0.0, 0.0)
        robot
    },
    { signal ->
        drive.localizer.pinpoint.update()

        drive.flMotor.power = signal
        drive.frMotor.power = signal
        drive.blMotor.power = signal
        drive.brMotor.power = signal

        arrayOf(drive.localizer.x, drive.localizer.y, drive.localizer.heading)
    },
    {
        signal, t -> t > 3.0
    },
    quasistaticSlope = 0.6,
    dynamicStep = 0.3,
    dynamicStepRate = 0.4
)