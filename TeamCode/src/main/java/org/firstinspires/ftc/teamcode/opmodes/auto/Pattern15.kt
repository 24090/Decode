package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.opmodes.commands.Auto
import org.firstinspires.ftc.teamcode.opmodes.poses.ShootPose
import org.firstinspires.ftc.teamcode.opmodes.poses.closeStartPose
import org.firstinspires.ftc.teamcode.opmodes.poses.getScoreDistance
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.util.storedPattern
import org.firstinspires.ftc.teamcode.util.storedPose

@Autonomous(name="Red - Close - 15 Sorted", group="Auto")
class Pattern15Red: Pattern15(true)

@Autonomous(name="Blue - Close - 15 Sorted", group="Push")
class Pattern15Blue: Pattern15(false)
open class Pattern15(isRed: Boolean): Auto(
    isRed,
    closeStartPose,
    {
        Race(
            Forever({
                recordTime("other")
                reads.update()
                storedPose = drive.localizer.pose
                storedPattern = indexTracker.pattern
                recordTime("reads")
                telemetry.addData("currentPose", drive.localizer.pose)
            }, "Reads" ),
            Sequence(
                Race(
                    Parallel(
                        intake.spinUp(),
                        Instant {
                            shooter.setHoodAngleAndVelocityFromDistance(ShootPose.Close.distance)
                            //camera.initPattern()
                        },
                        drive.goToCircle(ShootPose.Close.let { Pose(it.x, it.y, -0.2).mirroredIf(isRed) }),
                        Sleep(5.0)
                    ),
                    WaitUntil {
//                        camera.getPattern().let {
//                            if (it != null){
//                                indexTracker.pattern = it
//                                true
//                            } else {
//                                false
//                            }
//                        }
                        true
                    }
                ),
                closeShootCycle(),

                shooter.stop(),
                grabAndOpenCycleClose(),
                Instant{shooter.setHoodAngleAndVelocityFromDistance(ShootPose.Close.distance)},
                closeShootCycle(),

                shooter.stop(),
                gateIntakeCycleClose(),
                Instant{shooter.setHoodAngleAndVelocityFromDistance(ShootPose.Close.distance)},
                closeShootCyclePattern(),

                shooter.stop(),
                spikeIntakeCycleClose(0),
                Instant{shooter.setHoodAngleAndVelocityFromDistance(ShootPose.Close.distance)},
                closeShootCyclePattern(),

                shooter.stop(),
                spikeIntakeCycleClose(2),
                Instant{shooter.setHoodAngleAndVelocityFromDistance(getScoreDistance(Vector.fromCartesian(106.0, 12.0)))},
                leaveShootCyclePattern(),

                name = "Auto"
            ),
            Forever({
                drive.update(); recordTime("drive")
                shooter.update(); recordTime("shooter")
                intake.update(); recordTime("intake")
                telemetry.update()
            }, "Writes")
        )
    }
)