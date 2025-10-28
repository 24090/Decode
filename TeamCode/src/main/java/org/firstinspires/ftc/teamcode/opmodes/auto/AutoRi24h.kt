package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.drivetrain.Pose
import org.firstinspires.ftc.teamcode.opmodes.poses.closeDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.closePose
import org.firstinspires.ftc.teamcode.opmodes.poses.farDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.farPose
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.util.BulkReads
import kotlin.math.PI

@Autonomous(name = "Ri24h Auto")
class AutoRi24h: LinearOpMode() {
    override fun runOpMode() {
        val bulkReads = BulkReads(hardwareMap)
        val drive = Drive(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        val farShootCycle = {Sequence(
            drive.goTo(farPose),
            intake.spinUp(),
            shooter.waitForVelocity(),
            intake.releaseDual(),
            shooter.waitForVelocity(),
            intake.releaseDual(),
            name = "FarShootCycle"
        )}
        val closeShootCycle = {Sequence(
            drive.goTo(closePose),
            intake.spinUp(),
            shooter.waitForVelocity(),
            intake.releaseDual(),
            shooter.waitForVelocity(),
            intake.releaseDual(),
            name = "FarShootCycle"
        )}

        val grabBallCycle = {n: Int -> Sequence(
            intake.spinUp(),
            drive.goTo(Pose(36.0 + 24 * n, 24.0, PI/2), 8.0, 0.4),
            drive.goTo(Pose(36.0 + 24 * n, 50.0, PI/2)),
            intake.spinDown(),
            name = "GrabBallCycle $n"
        )}
        waitForStart()
        drive.localizer.pose = Pose(farPose.x, farPose.y, PI * -3/4)
        drive.targetPose = farPose
        runBlocking(Race(
            Forever {
                bulkReads.update()
                drive.update()
                shooter.update()
                intake.update()
            },
            Sequence(
                Instant{shooter.setTargetVelocityFromDistance(farDistance)},
                farShootCycle(),
                grabBallCycle(0),
                farShootCycle(),
                Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
                grabBallCycle(1),
                closeShootCycle(),
                grabBallCycle(2),
                closeShootCycle()
            )
        ))
    }
}
