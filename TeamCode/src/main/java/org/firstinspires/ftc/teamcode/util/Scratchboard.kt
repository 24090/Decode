package org.firstinspires.ftc.teamcode.util

/*
 SCRATCHBOARD:
    for running random tests and stuff
 */

import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.opmodes.poses.ShootPose
import org.firstinspires.ftc.teamcode.opmodes.poses.closeStartPose
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.sysid.SysIDRoutine
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.PurePursuitPath
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers.HeadingBehaviour
import kotlin.math.PI

fun main(){
    var t: Long = 0
    runBlocking(Race(
        Forever {println("loop")},
        ForeverCommand{
            Sequence(
                Instant{  t = System.currentTimeMillis() },
                Instant{},
                Sleep(0.0),
                Sequence(
                    Instant{},
                    Instant{},
                    Sleep(1e-12),
                ),
                Parallel(
                    Instant{},
                    Race(
                        Instant{},
                        Sequence(
                            Instant{}
                        ),
                        Sleep(1e-12),
                    ),
                    Sleep(1e-12),
                ),
                Race(
                    Instant{}
                ),
                Instant { println( System.currentTimeMillis() - t) },
            )
        }
    ))

}