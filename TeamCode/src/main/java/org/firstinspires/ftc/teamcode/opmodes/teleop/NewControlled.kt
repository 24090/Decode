package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Future
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.opmodes.commands.shootAll
import org.firstinspires.ftc.teamcode.opmodes.poses.getScoreAngle
import org.firstinspires.ftc.teamcode.opmodes.poses.getScoreDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.inLaunchZone
import org.firstinspires.ftc.teamcode.opmodes.poses.parkPose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.opmodes.poses.closeStartPose
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.getHeadingLockTeleop
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
import org.firstinspires.ftc.teamcode.util.predictedShootPosition
import org.firstinspires.ftc.teamcode.util.storedPattern
import org.firstinspires.ftc.teamcode.util.storedPose

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

        val getPredictedPosition = {predictedShootPosition(
            intake.getMinShootTime(),
            drive.localizer.pose,
            drive.localizer.poseVel,
            drive.estimateAcceleration()
        )}
        val setShooters = { position: Vector ->
            getScoreDistance(position, isRed.get()).let {
                shooter.targetVelocityLeft = shooter.distanceToVelocityLeftLUT.get(it)
                shooter.targetVelocityRight = shooter.distanceToVelocityRightLUT.get(it)
            }
        }

        val translationalFunction = getTeleopTranslational(gamepad1, drive.localizer, lastLockTranslational, targetPose, isRed)
        val normalFollow = getTeleopFollower(gamepad1, drive.localizer, isRed, lastLockHeading, targetPose, translationalFunction)
        val headingLockFollow = getHeadingLockTeleop({ getScoreAngle(getPredictedPosition(), isRed.get()) }, gamepad1, drive.localizer, translationalFunction, isRed, targetPose)


        val changeShootingMode = {
            shootingMode = !shootingMode
            drive.follow = if (shootingMode) headingLockFollow else normalFollow
            gamepad1.rumble(1.0, 1.0, 150)
        }

        drive.follow = normalFollow

        val relocalize = {
            val cameraPose = camera.getPose()
            if (cameraPose != null){
                drive.localizer.pose = drive.localizer.pose * 6.0/7.0 + cameraPose * 1.0/6.0
            }

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
                storedPose ?: closeStartPose.mirroredIf(isRed.get())
            else
                closeStartPose.mirroredIf(isRed.get())

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
                        drive.goToCircle(parkPose.mirroredIf(isRed.get())),
                    ),
                    Forever {
                        intake.update()
                        drive.update()
                        shooter.update()
                        telemetry.update()
                    }
                ))
                drive.follow = normalFollow
            }

            if (inLaunchZone(drive.localizer.pose) && shootingMode) {
                runBlocking(Race(
                    WaitUntil { gamepad1.leftBumperWasPressed() || !inLaunchZone(drive.localizer.pose, -2.0) },
                    Forever {
                        val packet = TelemetryPacket()
                        reads.update()
                        updateP2()
                    },
                    Sequence(
                        Instant {
                            targetPose.set(
                                getStopPosition(
                                    drive.localizer.pose,
                                    getRelativeVelocity(drive.localizer.pose, drive.localizer.poseVel).vector()
                                ).let {
                                    Pose(
                                        it.x,
                                        it.y,
                                        getScoreAngle(Vector.fromCartesian(it.x, it.y), isRed.get())
                                    )
                                }
                            )
                            shooter.setTargetVelocityFromDistance(getScoreDistance(targetPose.get().vector(), isRed.get()))
                        },
                        Future { drive.goToCircle(targetPose.get()) },
                        Parallel(
                            Instant {relocalize()} ,
                            shootAll(intake, shooter),
                        )
                    ),
                    Forever {
                        intake.update()
                        drive.update()
                        shooter.update()
                        telemetry.addData("Pose 2:", "${drive.localizer.pose} (${targetPose.get()})")
                        telemetry.addData("Shooter Velocity: ", "L: ${shooter.motorLeft.velocity}(${shooter.targetVelocityLeft}) R: ${shooter.motorRight.velocity}(${shooter.targetVelocityRight})")
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
            if (shootingMode) (
                setShooters(getPredictedPosition())
            ) else {
                shooter.targetVelocityLeft = 0.0
                shooter.targetVelocityRight = 0.0
            }
            shooter.update()
            intake.update()
            telemetry.addData("pose", drive.localizer.pose)
            telemetry.addData("Shooter Velocity: ", "L: ${shooter.motorLeft.velocity}(${shooter.targetVelocityLeft}) R: ${shooter.motorRight.velocity}(${shooter.targetVelocityRight})")
            telemetry.update()
        }
    }

}