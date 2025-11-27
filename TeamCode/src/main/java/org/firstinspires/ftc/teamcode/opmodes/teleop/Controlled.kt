package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.ForeverCommand
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.opmodes.commands.shootCycle
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Vector
import org.firstinspires.ftc.teamcode.opmodes.poses.scorePosition
import org.firstinspires.ftc.teamcode.opmodes.poses.startPose
import org.firstinspires.ftc.teamcode.opmodes.poses.storedPose
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherLeftBack
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherLeftForward
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherRightBack
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherRightForward
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherWait
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter

@TeleOp(name="Controlled")
class Controlled: LinearOpMode() {
    override fun runOpMode() {
        var isRed = false
        val reads = Reads(hardwareMap)
        val drive = Drive(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)

        var lastLockHeading = false
        var lastLockTranslational = false
        val isLockHeading = {
            gamepad1.right_stick_x.toDouble() == 0.0 && (drive.localizer.headingVel < 0.02 || lastLockHeading)
        }
        val isLockTranslational = {
            gamepad1.left_stick_x.toDouble() == 0.0 && gamepad1.left_stick_y.toDouble() == 0.0 && ((drive.localizer.xVel < 0.2 && drive.localizer.yVel < 0.2) || lastLockTranslational)
        }
        val updateHeadingOverride = {
            val lockHeading = isLockHeading()
            if (lockHeading){
                if (!lastLockHeading){
                    drive.targetPose.heading = drive.localizer.heading
                }
                drive.updateHeading()
            } else {
                drive.turn = -gamepad1.right_stick_x.toDouble()
            }
            lastLockHeading = lockHeading
        }
        val updateTranslationalOverride = {
            val lockTranslational = isLockTranslational()
            if (lockTranslational) {
                if (!lastLockTranslational) {
                    drive.targetPose.x = drive.localizer.x
                    drive.targetPose.y = drive.localizer.y
                }
                drive.updateTranslational()
            } else {
//                var v = Vector.fromCartesian(-gamepad1.left_stick_x.toDouble(), gamepad1.left_stick_y.toDouble())
//                if (isRed) v = v.rotated(PI)
//                drive.strafe = v.rotated(-drive.localizer.heading).y
//                drive.drive =  v.rotated(-drive.localizer.heading).x
                drive.drive = -gamepad1.left_stick_y.toDouble()
                drive.strafe = -gamepad1.left_stick_x.toDouble()
            }
            lastLockTranslational = isLockTranslational()
        }
        drive.currentUpdateHeading = updateHeadingOverride
        drive.currentUpdateTranslational = updateTranslationalOverride

        waitForStart()
        if (!opModeIsActive()){
            return
        }

        drive.localizer.pose = storedPose ?: startPose
        var time = System.currentTimeMillis()
        val recordTime = { name:String ->
            val newTime = System.currentTimeMillis()
            telemetry.addData("$name (ms)", newTime - time)
            time = newTime
        }

        while (opModeIsActive()){
            reads.update()
            recordTime("loop")
            if (gamepad1.backWasPressed()) {
                drive.localizer.pose = Pose(72.0, 0.0, 0.0)
                drive.targetPose = drive.localizer.pose
            }
            if (gamepad1.guideWasPressed()) {
                isRed = !isRed
            }
            if (gamepad1.aWasPressed()) {
                runBlocking(intake.spinUp())
            }
            if (gamepad1.bWasPressed()) {
                runBlocking(intake.stop())
            }
            if (gamepad1.leftBumperWasPressed()) {
                runBlocking(
                    ForeverCommand{
                        Sequence (
                            Instant{
                                intake.pusherLeft.position = pusherLeftForward
                                intake.pusherRight.position = pusherRightBack
                            },
                            Sleep(pusherWait),
                            Instant{
                                intake.pusherRight.position = pusherRightForward
                                intake.pusherLeft.position = pusherLeftBack
                            },
                            Sleep(pusherWait),
                        )
                    }
                )
            }
            if (gamepad1.rightBumperWasPressed()) {
                runBlocking(intake.releaseRight())
            }
            if (gamepad1.xWasPressed()) {
                drive.currentUpdateHeading = drive::updateHeading
                drive.currentUpdateTranslational = drive::updateTranslational
                runBlocking(Race(
                    WaitUntil { !gamepad1.x },
                    Forever {
                        reads.update()
                    },
                    Sequence(
                        Parallel(
                            WaitUntil { drive.error.heading < 0.04 },
                        ),
                        shootCycle(intake, shooter)
                    ),
                    Forever {
                        intake.update()
                        val relativePose = (scorePosition.mirroredIf(isRed) - Vector.fromPose(drive.localizer.pose))
                        drive.targetPose = Pose(
                            drive.localizer.x, drive.localizer.y,
                            relativePose.angle
                        )
                        drive.update()
                        shooter.setTargetVelocityFromDistance(relativePose.length)
                        shooter.update()
                        telemetry.addData("Shooter velocity", (shooter.motorLeft.velocity + shooter.motorRight.velocity)/2)
                        telemetry.addData("Target velocity", shooter.targetVelocity)
                        telemetry.update()
                    }
                ))
                runBlocking(intake.spinUp())
                intake.resetPushers()
                drive.currentUpdateHeading = updateHeadingOverride
                drive.currentUpdateTranslational = updateTranslationalOverride
            }
            if (gamepad1.yWasPressed()) {
                runBlocking(Race(
                    WaitUntil { !gamepad1.y },
                    Forever {
                        reads.update()
                    },
                    shootCycle(intake, shooter),
                    Forever {
                        intake.update()
                        val relativePose = (scorePosition.mirroredIf(isRed) - Vector.fromPose(drive.localizer.pose))
                        drive.update()
                        shooter.setTargetVelocityFromDistance(relativePose.length)
                        shooter.update()
                        telemetry.addData("Shooter velocity", (shooter.motorLeft.velocity + shooter.motorRight.velocity)/2)
                        telemetry.addData("Target velocity", shooter.targetVelocity)
                        telemetry.update()
                    }
                ))
                runBlocking(intake.spinUp())
                intake.resetPushers()
                drive.currentUpdateHeading = updateHeadingOverride
                drive.currentUpdateTranslational = updateTranslationalOverride
            }

            drive.update()
            shooter.setTargetVelocityFromDistance((scorePosition.mirroredIf(isRed) - Vector.fromPose(drive.localizer.pose)).length)
            shooter.update()
            intake.update()
            telemetry.addData("pose", drive.localizer.pose)
            telemetry.addData("scorePose", scorePosition.mirroredIf(isRed))
            telemetry.addData("distance", (scorePosition.mirroredIf(isRed) - Vector.fromPose(drive.localizer.pose)).length)
            telemetry.addData("Target velocity", shooter.targetVelocity)
            telemetry.update()
        }
    }

}