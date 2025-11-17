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
        val reads = Reads(hardwareMap)
        val drive = Drive(hardwareMap)
        var mirror = false
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        val updateHeadingOverride = {
            drive.turn = -gamepad1.right_stick_x.toDouble()
        }
        val updateTranslationalOverride = {
            drive.strafe = -gamepad1.left_stick_x.toDouble()
            drive.drive = -gamepad1.left_stick_y.toDouble()
        }
        drive.currentUpdateHeading = updateHeadingOverride
        drive.currentUpdateTranslational = updateTranslationalOverride
        waitForStart()

        if (!opModeIsActive()){
            return
        }

        drive.localizer.pose = storedPose ?: startPose

        while (opModeIsActive()){
            reads.update()

            if (gamepad1.backWasPressed()) {
                drive.localizer.pose = Pose(72.0, 0.0, 0.0)
            }
            if (gamepad1.guideWasPressed()) {
                mirror = !mirror
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
                            shooter.waitForVelocity(),
                            intake.fullAdjustThird(),
                        ),
                        intake.releaseDual(),
                        Sleep(1.0),
                        intake.spinUp(),
                        Sleep(0.5),
                        intake.releaseDual(),
                        intake.spinUp()
                    ),
                    Forever {
                        intake.update()
                        val relativePose = (scorePosition.mirroredIf(mirror) - Vector.fromPose(drive.localizer.pose))
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
                    Sequence(
                        Parallel(
                            shooter.waitForVelocity(),
                            intake.fullAdjustThird(),
                        ),
                        intake.releaseDual(),
                        Sleep(1.0),
                        intake.spinUp(),
                        Sleep(0.5),
                        intake.releaseDual(),
                        intake.spinUp()
                    ),
                    Forever {
                        intake.update()
                        val relativePose = (scorePosition.mirroredIf(mirror) - Vector.fromPose(drive.localizer.pose))
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
            shooter.setTargetVelocityFromDistance((scorePosition.mirroredIf(mirror) - Vector.fromPose(drive.localizer.pose)).length)
            shooter.update()
            intake.update()
            telemetry.addData("pose", drive.localizer.pose)
            telemetry.addData("distance", (scorePosition.mirroredIf(mirror) - Vector.fromPose(drive.localizer.pose)).length)
            telemetry.addData("Target velocity", shooter.targetVelocity)
            telemetry.update()
        }
    }

}