package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.opmodes.commands.grabBallCycle
import org.firstinspires.ftc.teamcode.opmodes.commands.loadZoneCycle
import org.firstinspires.ftc.teamcode.opmodes.commands.releasePattern
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.opmodes.poses.closeDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.closePose
import org.firstinspires.ftc.teamcode.opmodes.poses.farPose
import org.firstinspires.ftc.teamcode.opmodes.poses.getScoreDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.getScorePose
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.huskylens.HuskyLens
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.subsystems.vision.Camera
import org.firstinspires.ftc.teamcode.util.IndexTracker
import org.firstinspires.ftc.teamcode.util.calculatePredictiveMoveShoot
import org.firstinspires.ftc.teamcode.util.storedPattern
import org.firstinspires.ftc.teamcode.util.storedPose
import kotlin.math.PI

@Autonomous(name="AutoExperimentalRed", group="Auto")
class FullAutoRedExperimental: FullAutoExperimental(true)

@Autonomous(name="AutoExperimentalBlue", group="Auto")
class FullAutoBlueExperimental: FullAutoExperimental(false)
open class FullAutoExperimental(val isRed: Boolean): LinearOpMode() {
    override fun runOpMode() {
        val reads = Reads(hardwareMap)
        val drive = Drive(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        val huskyLens = HuskyLens(hardwareMap)
        val camera = Camera(hardwareMap)
        val indexTracker = IndexTracker()
        indexTracker.rampCount = 0
        val farShootCyclePattern = {Sequence(
            drive.goToCircle(farPose.mirroredIf(isRed), 2.0),
            releasePattern(intake,shooter,huskyLens,indexTracker),
            name = "FarShootCycle"
        )}
        val closeShootCyclePattern = {Sequence(
            drive.goToCircle(closePose.mirroredIf(isRed), 2.0),
            releasePattern(intake,shooter,huskyLens,indexTracker),
            name = "CloseShootCycle"
        )}
        val leaveShootCyclePattern = {Sequence(
            drive.goToCircle(getScorePose(Vector.fromCartesian(106.0, 12.0)).mirroredIf(isRed), 2.0),
            releasePattern(intake,shooter,huskyLens,indexTracker),
            name = "CloseShootCycle"
        )}
        val closeShootMovingCyclePattern = {Race(
            drive.goToCircle(getScorePose(Vector.fromCartesian(106.0, 12.0)).mirroredIf(isRed), 2.0),
            Sequence(
                WaitUntil{drive.inShootableZone()},
                releasePattern(intake,shooter,huskyLens,indexTracker))
        )}


        val postReleaseCycle = {Race(
            Sequence(
                intake.spinUp(),
                Sleep(0.3),
                intake.waitForStall()
            ),
            Sequence(
                Race(
                    drive.goToCircle(Pose(robotWidth/2.0 + 3.0, 48 + robotLength/2.0 - 3.0, PI/2.0).mirroredIf(isRed), 4.0),
                    Sleep(4.0)
                ),
                Sleep(0.3),
                Race(
                    drive.goToCircle(Pose(robotLength/2.0 + 3.0, 71.0 - robotWidth/2.0, 0.0).mirroredIf(isRed), 4.0),
                )
            )
        ) }

        while (opModeInInit()){
            indexTracker.pattern = camera.getPattern() ?: indexTracker.pattern
            telemetry.addData("pattern", indexTracker.pattern)
            telemetry.update()
        }
        waitForStart()

        if (!opModeIsActive()){
            return
        }

        drive.localizer.pose = Pose(robotLength/2.0, robotWidth/2.0, 0.0).mirroredIf(isRed)
        drive.startP2PWithTargetPose(closePose.mirroredIf(isRed))
        var time = System.currentTimeMillis()
        val recordTime = { name:String ->
            val newTime = System.currentTimeMillis()
            telemetry.addData("$name (ms)", newTime - time)
            time = newTime
        }

        runBlocking(Race(
            Forever({
                recordTime("other")
                reads.update();
                recordTime("reads")
                telemetry.addData("currentPose", drive.localizer.pose)
            }, "Reads" ),
            Sequence(
                Race(
                    Forever{
                        Parallel(Instant{shooter.targetVelocityLeft = shooter.exitVelocityToLeftVelocityLUT.get(
                        calculatePredictiveMoveShoot(0.0,drive.localizer.pose, drive.localizer.poseVel)?.first ?: 1400.0)}, Instant{shooter.targetVelocityRight = shooter.exitVelocityToRightVelocityLUT.get(
                                calculatePredictiveMoveShoot(0.0,drive.localizer.pose, drive.localizer.poseVel)?.first ?: 1400.0)})
                    },
                    closeShootMovingCyclePattern(),
                ),

                Instant{shooter.targetVelocityLeft = 0.0},
                Instant{shooter.targetVelocityRight = 0.0},
                grabBallCycle(2, isRed, intake, drive),
                Instant{shooter.setTargetVelocityFromDistance(closeDistance)},

                closeShootCyclePattern(),

                Instant{shooter.targetVelocityLeft = 0.0},
                Instant{shooter.targetVelocityRight = 0.0},
                grabBallCycle(1, isRed, intake, drive),
                Instant{shooter.setTargetVelocityFromDistance(closeDistance)},

                closeShootCyclePattern(),

                Instant{shooter.targetVelocityLeft = 0.0},
                Instant{shooter.targetVelocityRight = 0.0},
                grabBallCycle(0, isRed, intake, drive),
                Instant{shooter.setTargetVelocityFromDistance(closeDistance)},

                closeShootCyclePattern(),

                Instant{shooter.setTargetVelocityFromDistance(getScoreDistance(Vector.fromCartesian(96.0, 12.0)))},

                loadZoneCycle(isRed, intake, drive),

                leaveShootCyclePattern(),
                name = "Auto"
            ),
            Forever({
                drive.update(); recordTime("drive")
                shooter.update(); recordTime("shooter")
                intake.update(); recordTime("intake")
                telemetry.update()
            }, "Writes")
        ))
        storedPose = drive.localizer.pose
        storedPattern = indexTracker.pattern
    }
}
