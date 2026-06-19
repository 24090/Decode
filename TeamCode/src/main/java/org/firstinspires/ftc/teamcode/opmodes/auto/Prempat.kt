package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.opmodes.commands.Auto
import org.firstinspires.ftc.teamcode.opmodes.poses.ShootPose
import org.firstinspires.ftc.teamcode.opmodes.poses.closeStartPose

@TeleOp class PrempatRed(): Prempat(true)
@TeleOp class PrempatBlue(): Prempat(false)


open class Prempat(red: Boolean) : Auto(red, startPose = closeStartPose, { Race(
    Forever({
        recordTime("other")
        reads.update()
        recordTime("reads")
        telemetry.addData("currentPose", drive.localizer.pose)
    }, "Reads" ),
    Sequence(
        shootCycle(ShootPose.Close),
        spikeIntakeCycleClose(1),
        shootCycle(ShootPose.Close),
        gateIntakeCycle(ShootPose.Close),
        closeShootCyclePattern(),
        spikeIntakeCycleClose(2),
        closeShootCyclePattern(),
        spikeIntakeCycleClose(0),
        closeShootCyclePattern()
    ),
    Forever({
        drive.update(); recordTime("drive")
        shooter.update(); recordTime("shooter")
        intake.update(); recordTime("intake")
        telemetry.update()
    }, "Writes")
)})