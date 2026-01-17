package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.opmodes.commands.moveShootAll
import org.firstinspires.ftc.teamcode.opmodes.commands.shootAll
import org.firstinspires.ftc.teamcode.opmodes.poses.inLaunchZone
import org.firstinspires.ftc.teamcode.opmodes.poses.parkPose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.opmodes.poses.scorePosition
import org.firstinspires.ftc.teamcode.opmodes.poses.startPose
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.getMoveShootTeleop
import org.firstinspires.ftc.teamcode.subsystems.drive.getPointToPoint
import org.firstinspires.ftc.teamcode.subsystems.drive.getStopPosition
import org.firstinspires.ftc.teamcode.subsystems.drive.getTeleopFollower
import org.firstinspires.ftc.teamcode.subsystems.drive.getTeleopTranslational
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.getRelativeVelocity
import org.firstinspires.ftc.teamcode.subsystems.huskylens.HuskyLens
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.subsystems.vision.Camera
import org.firstinspires.ftc.teamcode.util.IndexTracker
import org.firstinspires.ftc.teamcode.util.Reference
import org.firstinspires.ftc.teamcode.util.calculatePredictiveMoveShoot
import org.firstinspires.ftc.teamcode.util.predictedShootPosition
import org.firstinspires.ftc.teamcode.util.storedPattern
import org.firstinspires.ftc.teamcode.util.storedPose
import kotlin.math.abs

@TeleOp(name="NewControlled")
class NewControlled: LinearOpMode() {
    override fun runOpMode() {
        val dashboard = FtcDashboard.getInstance()
        val isRed = Reference(false)
        var useStoredPose = false
        val lastLockHeading = Reference(false)
        val lastLockTranslational = Reference(false)
        val targetPose = Reference(Pose(0.0, 0.0, 0.0))
        var shootingMode = false

        val drive = Drive(hardwareMap)
        drive.localizer.pinpoint.recalibrateIMU()

        var time = System.currentTimeMillis()
        val recordTime = { name:String ->
            val newTime = System.currentTimeMillis()
            telemetry.addData("$name (ms)", newTime - time)
            time = newTime
        }

        while (opModeInInit()){
            if (gamepad2.guideWasPressed()) {
                isRed.set(!isRed.get())
            }
            if (gamepad2.backWasPressed()) {
                useStoredPose = !useStoredPose
            }
            telemetry.addData("isRed? (center button/guide)", isRed)
            telemetry.addData("useStoredPose? (back)", useStoredPose)
            telemetry.update()
        }

        val reads = Reads(hardwareMap)

        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        val camera = Camera(hardwareMap)
        val huskyLens = HuskyLens(hardwareMap)
        val indexTracker = IndexTracker()


        indexTracker.pattern = storedPattern ?: indexTracker.pattern
        camera.initLocalize()

        val translationalFunction = getTeleopTranslational(gamepad1, drive.localizer, lastLockTranslational, targetPose, isRed)
        val normalFollow = getTeleopFollower(gamepad1, drive.localizer, isRed, lastLockHeading, targetPose, translationalFunction)
        val moveShootFollow = getMoveShootTeleop({ (scorePosition.mirroredIf(isRed.get()) - drive.localizer.pose.vector()).angle }, gamepad1, drive.localizer, translationalFunction)

        val getPredictedPosition = {predictedShootPosition(
            intake.getMinShootTime(),
            drive.localizer.pose,
            drive.localizer.poseVel,
            drive.estimateAcceleration()
        )}
        val getPredictedAngle = { (scorePosition.mirroredIf(isRed.get()) - getPredictedPosition()).angle }
        val getPredictedDistance = {}
        val changeShootingMode = {
            shootingMode = !shootingMode
            drive.follow = if (shootingMode) moveShootFollow else normalFollow

            if (!shootingMode) {
                shooter.targetVelocityLeft = 1400.0
                shooter.targetVelocityRight = 1400.0
            }
        }

        drive.follow = normalFollow

        val updateLocalizer = {
//            val cameraPose = camera.getPose()
//            if (cameraPose != null){
//                val diff = Vector.fromPose(cameraPose - drive.localizer.pose).clampedLength(0.5)
//                drive.localizer.pose += Pose(diff.x, diff.y, 0.0)
//            }

        }

        val updateP2 = {
            if (gamepad2.guideWasPressed()) {
                isRed.set(!isRed.get())
            }
            if (gamepad2.xWasPressed()){
                indexTracker.rampCount = ((indexTracker.rampCount + 1)%9 + 9)%9
            }
            if (gamepad2.yWasPressed()){
                indexTracker.rampCount = ((indexTracker.rampCount - 1)%9 + 9)%9
            }
            telemetry.addLine("--------- FOR P2 ---------")
            telemetry.addData("isRed? (center button/guide)", isRed)
            telemetry.addData("rampCount (x and y)", indexTracker.rampCount)
            telemetry.addData("pattern", indexTracker.pattern)
            telemetry.addLine("--------------------------")
        }

        drive.localizer.pose =
            if (useStoredPose)
                storedPose ?: startPose.mirroredIf(isRed.get())
            else
                startPose.mirroredIf(isRed.get())

        while (opModeIsActive()){
            reads.update()
            updateP2()
            recordTime("loop")
            if (gamepad1.aWasPressed()) {
                runBlocking(intake.spinUp())
            }
            if (gamepad1.bWasPressed()) {
                runBlocking(intake.stop())
            }
            if (gamepad1.rightBumperWasPressed()){
                shooter.targetVelocityLeft = 0.0
                shooter.targetVelocityRight = 0.0
                runBlocking(Race(
                    WaitUntil { !gamepad1.right_bumper},
                    Forever {
                        reads.update()
                        updateP2()
                    },
                    Sequence(
                        drive.goToCircle(parkPose.mirroredIf(!isRed.get()) + Pose(-24.0, 0.0, 0.0)),
                        drive.doWheelie(intake)
                    ),
                    Forever {
                        intake.update()
                        drive.update()
                        shooter.update()
                        telemetry.addData("Shooter velocity", (shooter.motorLeft.velocity + shooter.motorRight.velocity)/2)
                        telemetry.addData("Target velocity: ", "L: ${shooter.targetVelocityLeft}, R: ${shooter.targetVelocityRight}")
                        telemetry.update()
                    }
                ))
                drive.follow = normalFollow
            }

            if (inLaunchZone(drive.localizer.pose) && shootingMode) {
                runBlocking(Race(
                    WaitUntil { gamepad1.leftBumperWasPressed() || !inLaunchZone(drive.localizer.pose) },
                    Forever {
                        val packet = TelemetryPacket()
                        reads.update()
                        updateP2()
                        val relativePose = (scorePosition.mirroredIf(isRed.get()) - Vector.fromPose(drive.localizer.pose))
                        shooter.setTargetVelocityFromDistance(relativePose.length)
                        (scorePosition.mirroredIf(isRed.get()) - Vector.fromPose(drive.localizer.pose)).length.let {
                            shooter.targetVelocityLeft = shooter.distanceToVelocityLeftLUT.get(it)
                            shooter.targetVelocityRight = shooter.distanceToVelocityRightLUT.get(it)
                        }
                        packet.put("left vel error", shooter.targetVelocityLeft - shooter.motorLeft.velocity)
                        packet.put("right vel error", shooter.targetVelocityRight - shooter.motorRight.velocity)
                    },
                    Sequence(
                        Instant {
                            targetPose.set(getStopPosition(drive.localizer.pose, getRelativeVelocity(drive.localizer.pose, drive.localizer.poseVel).vector()).let {
                                Pose(it.x, it.y, targetPose.get().heading)
                            })
                            drive.follow = getPointToPoint(targetPose, drive.localizer)
                        },
                        WaitUntil {drive.localizer.pose.inCircle(targetPose.get())},
                        shootAll(intake, shooter),
                        Instant {
                            gamepad1.rumble(100)
                        }
                    ),
                    Forever {
                        intake.update()
                        drive.update()
                        shooter.update()
                        telemetry.addData(
                            "Shooter velocity",
                            (shooter.motorLeft.velocity + shooter.motorRight.velocity) / 2
                        )
                        telemetry.addData(
                            "Shooter Velocity: ", "L: ${shooter.motorLeft.velocity} R: ${shooter.motorRight.velocity}",
                            (shooter.motorLeft.velocity + shooter.motorRight.velocity) / 2
                        )
                        telemetry.addData("Target velocity: ", "L: ${shooter.targetVelocityLeft}, R: ${shooter.targetVelocityRight}")
                        telemetry.update()
                    }
                )
                )
                changeShootingMode()
                intake.spinUp().update()
            }

            if (gamepad1.leftBumperWasPressed()){
                changeShootingMode()
            }

            drive.update()
            if (shootingMode) {
                shooter.targetVelocityLeft = shooter.exitVelocityToLeftVelocityLUT.get(moveShootOutputs?.first ?: 1400.0)
                shooter.targetVelocityRight = shooter.exitVelocityToRightVelocityLUT.get(moveShootOutputs?.first ?: 1400.0)
            }
            shooter.update()
            intake.update()
            telemetry.addData("targetPose", targetPose.get())
            telemetry.addData("pose", drive.localizer.pose)
            telemetry.addData("distance", (scorePosition.mirroredIf(isRed.get()) - Vector.fromPose(drive.localizer.pose)).length)
            telemetry.addData("Target velocity: ", "L: ${shooter.targetVelocityLeft}, R: ${shooter.targetVelocityRight}")
            telemetry.addData("gamepad1turn", gamepad1.right_stick_x)
            telemetry.addData("gamepad1strafe", gamepad1.left_stick_x)
            telemetry.addData("gamepad1drive", gamepad1.left_stick_y)
            telemetry.update()
        }
    }

}