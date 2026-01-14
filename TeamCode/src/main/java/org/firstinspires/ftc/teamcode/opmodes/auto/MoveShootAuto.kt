package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commands.Command
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.opmodes.commands.fromRampCycle
import org.firstinspires.ftc.teamcode.opmodes.commands.grabBallCycle
import org.firstinspires.ftc.teamcode.opmodes.commands.loadZoneCycle
import org.firstinspires.ftc.teamcode.opmodes.commands.moveShootAll
import org.firstinspires.ftc.teamcode.opmodes.commands.shootPattern
import org.firstinspires.ftc.teamcode.opmodes.commands.shootAll
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.opmodes.poses.closeDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.closePose
import org.firstinspires.ftc.teamcode.opmodes.poses.farDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.farPose
import org.firstinspires.ftc.teamcode.opmodes.poses.getScoreDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.getScorePose
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.getMoveShootPointToPoint
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

@Autonomous(name="AutoRed", group="Auto")
class MoveShootAutoRed: MoveShootAuto(true)

@Autonomous(name="AutoBlue", group="Auto")
class MoveShootAutoBlue: MoveShootAuto(false)
open class MoveShootAuto(val isRed: Boolean): LinearOpMode() {
    override fun runOpMode() {
        val reads = Reads(hardwareMap)
        val drive = Drive(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        val huskyLens = HuskyLens(hardwareMap)
        val camera = Camera(hardwareMap)
        val indexTracker = IndexTracker()
        var moveShootOutputs: Pair<Double, Double>? = null

        indexTracker.rampCount = 0
        val farShootCycle = {Sequence(
            drive.goToCircle(farPose.mirroredIf(isRed), 2.0),
            shootPattern(intake,shooter,huskyLens,indexTracker),
            name = "FarShootCycle"
        )}
        val closeShootCycle = {Sequence(
            Instant { drive.follow = getMoveShootPointToPoint(
                closePose.mirroredIf(isRed),
                drive.localizer,
                { moveShootOutputs?.second }
            )},
            moveShootAll(intake, shooter, {drive.localizer.heading}, { moveShootOutputs }),
            name = "CloseShootCycle"
        )}
        val leaveShootCycle = {Sequence(
            Instant { drive.follow = getMoveShootPointToPoint(
                getScorePose(Vector.fromCartesian(106.0, 12.0)).mirroredIf(isRed),
                drive.localizer,
                { moveShootOutputs?.second }
            )},
            moveShootAll(intake, shooter, {drive.localizer.heading}, { moveShootOutputs }),
            name = "CloseShootCycle"
        )}

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

        val withMoveShooterUpdates = { c: Command ->
            Race(
                Forever {
                    val vs = moveShootOutputs?.first
                    if (vs != null){
                        shooter.targetVelocityLeft = shooter.exitVelocityToLeftVelocityLUT.get(vs)
                        shooter.targetVelocityRight = shooter.exitVelocityToRightVelocityLUT.get(vs)
                    } else {
                        shooter.targetVelocityLeft = 1500.0
                        shooter.targetVelocityRight = 1500.0
                    }

                },
                c
            )
        }

        runBlocking(Race(
            Forever({
                recordTime("other")
                reads.update()
                recordTime("reads")
                telemetry.addData("currentPose", drive.localizer.pose)
                moveShootOutputs = calculatePredictiveMoveShoot(0.0, drive.localizer.pose, drive.localizer.poseVel, drive.estimateAcceleration().let { Pose(it.x, it.y, 0.0) })
            }, "Reads" ),
            Sequence(
                Instant{shooter.setTargetVelocityFromDistance(farDistance)},
                farShootCycle(),

                withMoveShooterUpdates(Sequence(
                    shooter.stop(),
                    grabBallCycle(1, isRed, intake, drive),
                    Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
                    closeShootCycle(),

                    shooter.stop(),
                    fromRampCycle(isRed, intake, drive),
                    Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
                    closeShootCycle(),

                    shooter.stop(),
                    fromRampCycle(isRed, intake, drive),
                    Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
                    closeShootCycle(),

                    shooter.stop(),
                    grabBallCycle(0, isRed, intake, drive),
                    Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
                    closeShootCycle(),

                    shooter.stop(),
                    grabBallCycle(2, isRed, intake, drive),
                    Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
                    leaveShootCycle(),
                )),
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
