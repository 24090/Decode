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
import org.firstinspires.ftc.teamcode.opmodes.poses.farStartPose
import org.firstinspires.ftc.teamcode.util.storedPattern
import org.firstinspires.ftc.teamcode.util.storedPose

@Autonomous(name="Red - Far - 15", group="Auto")
class Far12Red: Far12(true)

@Autonomous(name="Blue - Far - 15", group="Auto")
class Far12Blue: Far12(false)

open class Far12(isRed: Boolean): Auto(
    isRed,
    farStartPose,
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
                            shooter.setHoodAngleAndVelocityFromDistance(ShootPose.Far.distance)
                            //camera.initPattern()
                        },
                        drive.goToCircle(ShootPose.Far.mirroredIf(isRed)),
                        Sleep(5.0)
                    ),
                    WaitUntil {
//                        camera.getPattern().let {
//                            if (it != null) {
//                                indexTracker.pattern = it
//                                true
//                            } else {
//                                false
//                            }
//                        }
                        true
                    }
                ),
                farShootCycle(),

                shooter.stop(),
                grabAndOpenCycleFar(),
                Instant { shooter.setHoodAngleAndVelocityFromDistance(ShootPose.Far.distance) },
                farShootCycle(),

                shooter.stop(),
                gateIntakeCycleFar(),
                Instant { shooter.setHoodAngleAndVelocityFromDistance(ShootPose.Far.distance) },
                farShootCycle(),

                shooter.stop(),
                spikeIntakeCycleFar(0),
                Instant { shooter.setHoodAngleAndVelocityFromDistance(ShootPose.Far.distance) },
                farShootCycle(),

                shooter.stop(),
                Instant { shooter.setHoodAngleAndVelocityFromDistance(ShootPose.Far.distance) },
                loadZoneCycle(),
                farShootCycle(),

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