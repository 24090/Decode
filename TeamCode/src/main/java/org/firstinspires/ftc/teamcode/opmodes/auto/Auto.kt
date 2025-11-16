package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.drivetrain.Pose
import org.firstinspires.ftc.teamcode.opmodes.poses.closeDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.closePose
import org.firstinspires.ftc.teamcode.opmodes.poses.farPose
import org.firstinspires.ftc.teamcode.opmodes.poses.storedPose
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import kotlin.math.PI

@Autonomous(name="AutoRed", group="Auto")
class AutoRed: Auto(true)

@Autonomous(name="AutoBlue", group="Auto")
class AutoBlue: Auto(false)

@Autonomous(name="AutoLeaveBlue", group="Auto")
class AutoLeaveBlue: AutoLeave(false)

@Autonomous(name="AutoLeaveRed", group="Auto")
class AutoLeaveRed: AutoLeave(true)

open class AutoLeave(val isRed: Boolean): LinearOpMode() {
    override fun runOpMode() {
        val drive = Drive(hardwareMap)
        val reads = Reads(hardwareMap)
        waitForStart()
        drive.localizer.pose = Pose(9.0,9.0,0.0).mirroredIf(isRed)
        drive.targetPose = farPose.mirroredIf(isRed)
        runBlocking(Race(
            Forever {
                reads.update()
                drive.update()
            },
            drive.goTo(Pose(26.0, 26.0, 26.0).mirroredIf(isRed)),
            Sleep(1.0)
        ))
        storedPose = drive.localizer.pose
    }
}


open class Auto(val isRed: Boolean): LinearOpMode() {
    override fun runOpMode() {
        val reads = Reads(hardwareMap)
        val drive = Drive(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        val farShootCycle = {Sequence(
            drive.goTo(farPose.mirroredIf(isRed)),
            intake.fullAdjustThird(),
            shooter.waitForVelocity(),
            intake.releaseDual(),
            Sleep(1.0),
            intake.spinUp(),
            Sleep(0.5),
            shooter.waitForVelocity(),
            intake.releaseDual(),
            name = "FarShootCycle"
        )}
        val closeShootCycle = {Sequence(
            drive.goTo(closePose.mirroredIf(isRed)),
            intake.fullAdjustThird(),
            shooter.waitForVelocity(),
            intake.releaseDual(),
            name = "CloseShootCycle"
        )}

        val grabBallCycle = {n: Int -> Sequence(
            intake.spinUp(),
            drive.goTo(Pose(36.0 + 24 * n, 24.0, PI/2).mirroredIf(isRed), 5.0, PI/2),
            drive.goTo(Pose(36.0 + 24 * n, if (n == 2) 50.0 else 50.0, PI/2).mirroredIf(isRed)),
            intake.stop(),
            name = "GrabBallCycle $n"
        )}

        waitForStart()

        drive.localizer.pose = Pose(9.0, 8.0, 0.0).mirroredIf(isRed)
        drive.targetPose = closePose.mirroredIf(isRed)
        var time = System.currentTimeMillis()
        val recordtime = { name:String ->
            val newtime = System.currentTimeMillis()
            telemetry.addData("$name (ms)", newtime - time)
            time = newtime
        }

        runBlocking(Race(
            Forever( {
                recordtime("other")
                reads.update(); recordtime("reads")
                drive.update(); recordtime("drive")
                shooter.update(); recordtime("shooter")
                intake.update(); recordtime("intake")
                telemetry.update()
            }, "Updates" ),
            Sequence(
                Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
                intake.spinUp(),
                closeShootCycle(),
                grabBallCycle(2),
                closeShootCycle(),
                Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
                grabBallCycle(1),
                closeShootCycle(),
                grabBallCycle(0),
                closeShootCycle(),
                name = "Auto"
            )
        ))
        storedPose = drive.localizer.pose
    }
}
