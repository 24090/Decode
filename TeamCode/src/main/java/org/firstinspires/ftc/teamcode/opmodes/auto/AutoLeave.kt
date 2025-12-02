package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.opmodes.poses.farPose
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.vision.Camera
import org.firstinspires.ftc.teamcode.util.IndexTracker
import org.firstinspires.ftc.teamcode.util.storedPattern
import org.firstinspires.ftc.teamcode.util.storedPose

@Autonomous(name="AutoLeaveBlue", group="Auto")
class AutoLeaveBlue: AutoLeave(false)

@Autonomous(name="AutoLeaveRed", group="Auto")
class AutoLeaveRed: AutoLeave(true)

open class AutoLeave(val isRed: Boolean): LinearOpMode() {
    override fun runOpMode() {
        val drive = Drive(hardwareMap)
        val reads = Reads(hardwareMap)
        val camera = Camera(hardwareMap)
        val indexTracker = IndexTracker()
        indexTracker.rampCount = 0
        camera.initPattern()
        waitForStart()
        while (opModeInInit()) {
            indexTracker.pattern = camera.getPattern() ?: indexTracker.pattern
            telemetry.addData("pattern", indexTracker.pattern)
            telemetry.update()
        }
        if (!opModeIsActive()) {
            return
        }

        drive.localizer.pose = Pose(robotLength/2.0,robotWidth/2.0,0.0).mirroredIf(isRed)
        drive.targetPose = farPose.mirroredIf(isRed)
        runBlocking(Race(
            Forever {
                reads.update()
            },
            Sequence(
                drive.goToCircle(Pose(26.0, 26.0, 26.0).mirroredIf(isRed)),
                Sleep(2.0)
            ),
            Forever {
                drive.update()
            }
        ))
        storedPose = drive.localizer.pose
        storedPattern = indexTracker.pattern
    }
}
