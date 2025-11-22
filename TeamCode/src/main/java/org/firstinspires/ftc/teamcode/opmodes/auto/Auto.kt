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
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.opmodes.poses.closeDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.closePose
import org.firstinspires.ftc.teamcode.opmodes.poses.farPose
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.opmodes.poses.storedPose
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import kotlin.math.PI
import kotlin.math.abs

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

        if (!opModeIsActive()){
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
    }
}


open class Auto(val isRed: Boolean): LinearOpMode() {
    override fun runOpMode() {
        val reads = Reads(hardwareMap)
        val drive = Drive(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        val shootCycle = {Sequence(
            intake.fullAdjustThird(),
            shooter.waitForVelocity(),
            intake.releaseDual(),
            Sleep(1.0),
            intake.spinUp(),
            Parallel(
                intake.waitForDistance(200),
                shooter.waitForVelocity(),
            ),
            intake.releaseDual(),
            name = "ShootCycle"
        )}
        val farShootCycle = {Sequence(
            drive.goToCircle(farPose.mirroredIf(isRed), 2.0),
            shootCycle(),
            name = "FarShootCycle"
        )}
        val closeShootCycle = {Sequence(
            drive.goToCircle(closePose.mirroredIf(isRed), 2.0),
            shootCycle(),
            name = "CloseShootCycle"
        )}

        val grabBallCycle = {n: Int -> Sequence(
            intake.spinUp(),
            drive.goToSquare(Pose(36.0 + 24.0 * n, 24.0, PI/2).mirroredIf(isRed), 1.0, 6.0),
            Instant {Drive.DriveConstants.kDT *= 2},
            Race(
                drive.goToCircle(Pose(
                    36.0 + 24.0 * n,
                    if (n==0) 65.0 else 72.0 - robotLength/2.0,
                    PI/2
                ).mirroredIf(isRed)),
                Sequence(
                    Sleep(0.3),
                    intake.waitForStall()
                ),
                Sequence(
                    Sleep(0.3),
                    WaitUntil{ abs(drive.localizer.yVel) < 0.3 }
                ),
            ),
            Instant {Drive.DriveConstants.kDT /= 2},
            name = "GrabBallCycle $n"
        )}

        waitForStart()

        if (!opModeIsActive()){
            return
        }

        drive.localizer.pose = Pose(robotWidth/2.0, robotLength/2.0, 0.0).mirroredIf(isRed)
        drive.targetPose = closePose.mirroredIf(isRed)
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
                telemetry.addData("targetPose", drive.targetPose)
                telemetry.addData("currentPose", drive.localizer.pose)
            }, "Reads" ),
            Sequence(
                Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
                closeShootCycle(),
                Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
                grabBallCycle(2),
                Parallel(
                    Sequence(Sleep(0.5), intake.stop()),
                    closeShootCycle(),
                ),
                Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
                grabBallCycle(1),
                Parallel(
                    Sequence(Sleep(0.5), intake.stop()),
                    closeShootCycle(),
                ),
                grabBallCycle(0),
                Parallel(
                    Sequence(Sleep(0.5), intake.stop()),
                    closeShootCycle(),
                ),
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
    }
}
