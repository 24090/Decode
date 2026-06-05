package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.ForeverCommand
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.opmodes.commands.Auto
import org.firstinspires.ftc.teamcode.opmodes.poses.ShootPose
import org.firstinspires.ftc.teamcode.opmodes.poses.farStartPose

@Autonomous(name="FarAutoRed", group="Auto")
class FarRed: Far(true)

@Autonomous(name="FarAutoBlue", group="Auto")
class FarBlue: Far(false)

open class Far(val isRed: Boolean): Auto(isRed, farStartPose, {Race(
    Forever({
        recordTime("other")
        reads.update()
        recordTime("reads")
        telemetry.addData("currentPose", drive.localizer.pose)
    }, "Reads" ),
    Sequence(
            drive.goToCircle(ShootPose.Far.mirroredIf(red), 3.0, 0.03),
            shootAll(ShootPose.Far.distance),

            spikeIntakeCycleFar(0),
            farShootCycle(),

            loadZoneCycle(),
            farShootCycle(),

            spikeIntakeCycleFar(0),
            farShootCycle(),

            loadZoneCycle(),
            farShootCycle(),

            spikeIntakeCycleFar(0),
            farShootCycle(),

            loadZoneCycle(),
            farShootCycle(),
    ),
    Forever({
        drive.update(); recordTime("drive")
        shooter.update(); recordTime("shooter")
        intake.update(); recordTime("intake")
        telemetry.update()
    }, "Writes")
)})
