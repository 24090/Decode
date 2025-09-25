package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.drivetrain.Pose
import org.firstinspires.ftc.teamcode.drivetrain.Vector
import org.firstinspires.ftc.teamcode.opmodes.poses.scorePosition
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.util.BulkReads

@TeleOp(name="Controlled")
class Controlled: LinearOpMode() {
    override fun runOpMode() {
        val bulkReads = BulkReads(hardwareMap)
        val drive = Drive(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        val driveFunction = {
            drive.strafe = gamepad1.left_stick_x.toDouble();
            drive.drive = -gamepad1.left_stick_y.toDouble();
            drive.turn = gamepad1.right_stick_x.toDouble();
            drive.setMotorPowers()
        }
        waitForStart()
        while (opModeIsActive()){
            bulkReads.update()
            driveFunction()
            shooter.setTargetVelocityFromDistance((scorePosition - Vector.fromPose(drive.localizer.pose)).length)
            shooter.update()
            intake.update()
            telemetry.addData("Shooter velocity", shooter.currentVelocity)
            telemetry.update()
            if (gamepad1.aWasReleased()) {
                runBlocking(intake.spinUp())
            }
            if (gamepad1.bWasReleased()) {
                runBlocking(intake.spinDown())
            }
            if (gamepad1.xWasReleased()) {
                runBlocking(Race(
                    Forever {
                        bulkReads.update()
                        drive.update()
                        shooter.update()
                        intake.update()
                        telemetry.addData("Shooter velocity", shooter.currentVelocity)
                        telemetry.update()
                    },
                    Instant {
                        val relativePose = (scorePosition - Vector.fromPose(drive.localizer.pose))
                        shooter.setTargetVelocityFromDistance(relativePose.length)
                        drive.targetPose = Pose(
                            drive.localizer.x, drive.localizer.y,
                            relativePose.angle
                        )
                    },
                    WaitUntil { drive.atTargetCircle(0.4, 0.04) },
                    shooter.waitForVelocity(),
                    intake.releaseBall()
                ))
            }
        }
    }

}