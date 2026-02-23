package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.opmodes.commands.Auto
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.PurePursuitPath
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers.HeadingBehaviour
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers.getPurePursuit
import kotlin.math.PI

@TeleOp
class PurePursuitTest: Auto(false, Pose(0.0, 0.0, 0.0), { Race(
    Forever {
        reads.update()
        drive.update()
    },
    Sequence(
        Instant {
            drive.follow = getPurePursuit(
                PurePursuitPath(
                    listOf(
                        Pose(0.0, 0.0, 0.0),
                        Pose(50.0, 0.0, PI/2),
                        Pose(50.0, 50.0, PI),
                        Pose(0.0, 50.0, -PI/2),
                        Pose(0.0, 0.0, 0.0),
                    ),
                    listOf(
                        HeadingBehaviour.Tangent(0.0),
                        HeadingBehaviour.Tangent(0.0),
                        HeadingBehaviour.Tangent(0.0),
                        HeadingBehaviour.Tangent(0.0),
                    ),
                    listOf(
                        25.0,
                        25.0,
                        25.0,
                        25.0
                    )
                ),
                drive.localizer
            )
        },
        Sleep(30.0)
    )
)})