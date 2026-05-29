package org.firstinspires.ftc.teamcode.opmodes.commands

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.commands.Command
import org.firstinspires.ftc.teamcode.commands.Future
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.opmodes.poses.ShootPose
import org.firstinspires.ftc.teamcode.opmodes.poses.getScorePose
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.PurePursuitPath
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers.HeadingBehaviour
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers.pointToPoint
import org.firstinspires.ftc.teamcode.subsystems.huskylens.HuskyLens
import org.firstinspires.ftc.teamcode.subsystems.huskylens.Lights
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherWait
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.subsystems.vision.Camera
import org.firstinspires.ftc.teamcode.util.IndexTracker
import org.firstinspires.ftc.teamcode.util.Observation
import org.firstinspires.ftc.teamcode.util.Pattern
import org.firstinspires.ftc.teamcode.util.Reference
import org.firstinspires.ftc.teamcode.util.clamp
import org.firstinspires.ftc.teamcode.util.storedRed
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.min

open class Robot(hwMap: HardwareMap, telemetry: Telemetry) {
    val telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    var red
        get() = storedRed.get()
        set(v) = storedRed.set(v)
    val reads = Reads(hwMap)
    val drive = Drive(hwMap)
    val shooter = Shooter(hwMap)
    val intake = Intake(hwMap)
    //val huskyLens = HuskyLens(hwMap)
    //val camera = Camera(hwMap)
    val lights = Lights(hwMap)
    val indexTracker = IndexTracker()

    var time = System.currentTimeMillis()
    fun recordTime(name: String) {
        val newTime = System.currentTimeMillis()
        telemetry.addData("$name (ms)", newTime - time)
        time = newTime
    }

    fun moveShootAll(getHeading: () -> Double, getMoveShootOutputs: () -> Pair<Double, Double>?) =
        org.firstinspires.ftc.teamcode.commands.Sequence(
            intake.fullAdjustThird(),
            WaitUntil {
                abs(getHeading() - (getMoveShootOutputs()?.second ?: 10.0)) < 0.06
                && shooter.velocitiesInThreshold(50.0)
            },
            intake.releaseDual(),
            Sleep(pusherWait),
            intake.spinUp(),
            Sleep(1.0),
            WaitUntil {
                abs(getHeading() - (getMoveShootOutputs()?.second ?: 10.0)) < 0.06
                && shooter.velocitiesInThreshold(50.0)
            },
            intake.releaseDual(),
        )
    fun shootAll() = Sequence(
        Sequence(
            shooter.waitForVelocity(),
            Parallel(
                intake.releaseDual(),
                intake.setAdjustThird()
            ),
            Sleep(pusherWait/2),
            Parallel(
                Instant { intake.behaviour = Intake.IntakeBehaviour.HyperGreedy },
                shooter.waitForVelocity(),
                Sleep(0.05),
            ),
            intake.releaseDual(),
            Instant {
                intake.behaviour = Intake.IntakeBehaviour.Grab
                intake.stallTest.resetCount()
            },
        ),
        name = "ShootCycle"
    )
    fun shootAll(distance: Double) = shootAll(shooter.distanceToVelocityLeftLUT.get(distance), shooter.secondaryDistanceToVelocityLeftLUT.get(distance), shooter.distanceToAngleLUT.get(distance), shooter.secondaryDistanceToAngleLUT.get(distance))
    fun shootAll(velocity1: Double, velocity2: Double, hood1: Double, hood2: Double) = Sequence(
        Sequence(
            Parallel(
                Instant{ shooter.setTargetVelocities(velocity1); shooter.setHoodAngles(hood1)},
                shooter.waitForVelocity()
            ),
            Parallel(
                intake.releaseDual(),
                intake.setAdjustThird()
            ),
            Sleep(pusherWait/2),
            Parallel(
                Instant {
                    shooter.setTargetVelocities(velocity2);
                    shooter.setHoodAngles(hood2)
                    intake.behaviour = Intake.IntakeBehaviour.HyperGreedy
                },
                shooter.waitForVelocity(),
                Sleep(0.05),
            ),
            intake.releaseDual(),
            Instant {
                intake.behaviour = Intake.IntakeBehaviour.Grab
                intake.stallTest.resetCount()
            },
        ),
        name = "ShootCycle"
    )
    fun shootPattern(): Command = Future {
        //huskyLens.update()
        val reload = {
            Parallel(
                shooter.waitForVelocity(),
                intake.spinUp(),
                Sleep(0.3),
            )
        }
        val held = Pattern.GPP //huskyLens.getHeldPattern()
        val pattern = indexTracker.getRecommendations()
        lights.turnOn()
        //val startCount = shooter.shootCounterLeft.count + shooter.shootCounterRight.count
        val indexWait = 0.75
        val command = Sequence(
            Parallel(
                intake.setAdjustThird(),
                shooter.waitForVelocity(),
            ),
            when (Pair(held, pattern)) {
                Pair(Pattern.GPP, Pattern.GPP) -> Sequence(
                    // left right+3
                    intake.releaseLeft(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseDual()
                )

                Pair(Pattern.GPP, Pattern.PGP) -> Sequence(
                    // right left 3
                    intake.releaseRight(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseLeft(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseDual()
                )

                Pair(Pattern.GPP, Pattern.PPG) -> Sequence(
                    // right 3 left
                    intake.releaseRight(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseRight(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseDual()
                )

                Pair(Pattern.PGP, Pattern.GPP) -> Sequence(
                    // right left+3
                    intake.releaseRight(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseDual()
                )

                Pair(Pattern.PGP, Pattern.PGP) -> Sequence(
                    // left right 3
                    intake.releaseLeft(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseRight(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseDual()
                )

                Pair(Pattern.PGP, Pattern.PPG), Pair(Pattern.PPG, Pattern.PGP) -> Sequence(
                    // left 3 right
                    intake.releaseLeft(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseLeft(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseDual()
                )

                Pair(Pattern.PPG, Pattern.GPP), Pair(Pattern.PPG, Pattern.PPG) -> Sequence(
                    // BAD/GOOD
                    // left+right 3
                    intake.releaseDual(),
                    reload(),
                    intake.releaseDual(),
                )

                else -> {
                    throw UnsupportedOperationException("Unreachable")
                }
            },
            Instant {
                lights.turnOff()
                intake.stallTest.resetCount()
            }
//            Instant {
//                indexTracker.processObservation(Observation.Shot(shooter.shootCounterLeft.count + shooter.shootCounterRight.count - startCount))
//            }
        )
        return@Future command
    }
    fun grabAndOpenCycleClose() = Sequence(
        intake.spinUp(),
        Race(
            drive.followPath(PurePursuitPath(
                listOf(ShootPose.Close, Pose(58.0, 25.0, PI/2.0).mirroredIf(red), Pose(58.0, 65.0, PI/2.0).mirroredIf(red)),
                listOf(HeadingBehaviour.Tangent(0.0), HeadingBehaviour.Tangent(0.0)),
                listOf(30.0, 20.0)
            )),
            WaitUntil { intake.isStalling() && drive.localizer.pose.mirroredIf(red).y > 48 },
            WaitUntil{ drive.localizer.poseVel.vector().length < 0.2 && drive.localizer.pose.mirroredIf(red).y > 48}
        ),
        drive.goToSquare(Pose(62.0, 48.0, PI/2.0).mirroredIf(red), 1.5, 1.5, 0.1),
        Race(
            drive.goToSquare(Pose(65.0, 54.0, PI/2.0).mirroredIf(red)),
            Sleep(0.75)
        ),
        drive.followPath(PurePursuitPath(
            listOf(Pose(65.0, 54.0, PI/2.0).mirroredIf(red), Pose(65.0, 14.0, PI/2.0).mirroredIf(red), ShootPose.Close.mirroredIf(red)),
            listOf(HeadingBehaviour.Tangent(0.0), HeadingBehaviour.Interpolate),
            listOf(40.0, 40.0)
        ), 5.0, 0.1),
    )
    fun grabAndOpenCycleFar() = Sequence(
        intake.spinUp(),
        Race(
            drive.followPath(PurePursuitPath(
                listOf(
                    Pose(62.0, 30.0, PI/2).mirroredIf(red),
                    Pose(66.0, 60.0, PI/2).mirroredIf(red)
                ),
                listOf(HeadingBehaviour.Tangent(0.0), HeadingBehaviour.Tangent(0.0)),
                listOf(40.0)
            )),
            WaitUntil { intake.isStalling() && drive.localizer.pose.mirroredIf(red).y > 48 },
            WaitUntil{ drive.localizer.poseVel.vector().length < 0.2 && drive.localizer.pose.mirroredIf(red).y > 48},
        ),
        drive.followPath(PurePursuitPath(
            listOf(
                Pose(60.0, 30.0, PI/2).mirroredIf(red),
                ShootPose.Far.mirroredIf(red),
            ),
            listOf(HeadingBehaviour.Tangent(PI)),
            listOf(30.0)
        ))
    )
    fun spikeIntakeCycleClose(n: Int) = spikeIntakeCycle(n, ShootPose.Close)
    fun spikeIntakeCycleFar(n: Int) = spikeIntakeCycle(n, ShootPose.Far)

    fun spikeIntakeCycle(n: Int, shootPose: ShootPose): Sequence{
        val endPoint = Pose(
            (if (n == 1) 34.0 else 36.0) + 24.0 * n,
            (65.0),
            PI/2
        )
        val shootPoint = shootPose
        return Sequence(
            Instant { intake.behaviour = Intake.IntakeBehaviour.Greedy},
            Race(
                drive.followPath(PurePursuitPath(
                    listOf(
                        Pose(
                            endPoint.x,
                            15.0,
                            PI/2
                        ).mirroredIf(red),
                        Pose(
                            endPoint.x,
                            45.0,
                            PI/2
                        ).mirroredIf(red),
                        endPoint.mirroredIf(red)
                    ),
                    listOf(HeadingBehaviour.Tangent(0.0), HeadingBehaviour.Tangent(0.0)),
                    listOf(20.0, 40.0),
                )),
                WaitUntil { intake.isStalling() && drive.localizer.pose.mirroredIf(red).y > 48 },
                WaitUntil{ drive.localizer.poseVel.vector().length < 1.0 && drive.localizer.pose.mirroredIf(red).y > 48},
            ),
            Instant{shooter.setHoodAngleAndVelocityFromDistance(shootPose.distance)},
            Race(
                drive.followPath(PurePursuitPath(
                    listOf(
                        Pose(endPoint.x, if (n == 1) 20.0 else 40.0, PI/2).mirroredIf(red),
                        shootPoint.mirroredIf(red)
                    ),
                    listOf(
                        HeadingBehaviour.Interpolate,
                    ),
                    listOf(
                        40.0
                    ),
                )),
                WaitUntil{drive.localizer.pose.inCircle(shootPoint.mirroredIf(red), 5.0, 1.0)},
            ),
            name = "GrabBallCycle $n"
        )
    }

    fun gateIntakeCycleClose() = gateIntakeCycle(ShootPose.Close)
    fun gateIntakeCycleFar() = gateIntakeCycle(ShootPose.Far)

    fun gateIntakeCycle(shootPose: ShootPose) = Sequence(
        Race(
            Sequence(
                Sleep(0.3),
                WaitUntil{drive.localizer.poseVel.vector().length < 0.5},
            ),
            drive.followPath(
                PurePursuitPath(
                    listOf(
                        shootPose.mirroredIf(red),
                        Pose(
                            59.4 + 0.5,
                            47.0,
                            1.05
                        ).mirroredIf(red),
                        Pose(
                                59.4 + 1.0,
                                56.12 + 1.3,
                                1.05
                        ).mirroredIf(red),
                    ),
                    listOf(
                        HeadingBehaviour.Tangent(0.0),
                        HeadingBehaviour.Tangent(0.0),
                    ),
                    listOf(
                        40.0,
                        40.0
                    ),
                )
            )

        ),
        Race(
            Sequence(
                intake.waitForStall(),
            ),
            Sequence(
                Sleep(1.4),
                drive.goToCircle(
                    Pose(
                        59.4 - 2,
                        56.12 - 1.0,
                        0.95
                    ).mirroredIf(red),
                ),
                drive.goToCircle(
                    Pose(
                        59.4,
                        56.12 + 2.0,
                        PI/2
                    ).mirroredIf(red),
                ),
            ),
            Sleep(3.2)

        ),
        Instant {
            indexTracker.processObservation(Observation.GateOpened)
            intake.behaviour = Intake.IntakeBehaviour.Grab
            shooter.setHoodAngleAndVelocityFromDistance(shootPose.distance)

        },
        Race(
            drive.followPath(PurePursuitPath(
                listOf(
                    Pose(
                        59.4,
                        56.12 + 1.85,
                        1.06
                    ).mirroredIf(red),
                    Pose(
                        59.4,
                        35.0,
                        PI/2
                    ).mirroredIf(red),
                    shootPose.mirroredIf(red)
                ),
                listOf(
                    HeadingBehaviour.Tangent(PI),
                    HeadingBehaviour.Interpolate
                ),
                listOf(
                    30.0,
                    40.0
                )
            )),
            WaitUntil {drive.localizer.pose.inCircle(shootPose.mirroredIf(red), 5.0, 0.1)}
        )
    )

    fun loadZoneCycle() = Race(
        Sequence(
            intake.spinUp(),
            Sleep(2.0),
            intake.waitForStall()
        ),
        Sequence(
            Sleep(2.0),
            WaitUntil { drive.localizer.poseVel.vector().length < 0.1}
        ),
        Sequence(
            drive.goToCircle(
                Pose(
                    robotWidth / 2.0 + 0.5,
                    72.0 - robotLength / 2.0 + 1.0,
                    PI / 2.0
                ).mirroredIf(red),
            )
        )
    )

    fun closeShootCyclePattern() = Sequence(
        Instant{lights.turnOn()},
        drive.goToCircle(ShootPose.Close.mirroredIf(red), 2.0, 0.05),
        shootPattern(),
        Instant{lights.turnOff()},
        name = "CloseShootCycle"
    )

    fun closeShootCycle() = Sequence(
        drive.goToCircle(ShootPose.Close.mirroredIf(red), 3.0, 0.06),
        shootAll(ShootPose.Close.distance),
        name = "CloseShootCycle"
    )

    fun farShootCycle() = Sequence(
        drive.goToCircle(ShootPose.Far.mirroredIf(red), 3.0, 0.03),
        shootAll(ShootPose.Far.distance),
        name = "FarShootCycle"
    )

    fun leaveShootCyclePattern() = Sequence(
        Instant{lights.turnOn()},
        drive.goToCircle(getScorePose(Vector.fromCartesian(106.0, 12.0)).mirroredIf(storedRed.get()), 2.0),
        shootPattern(),
        Instant{lights.turnOff()},
        name = "CloseShootCycle"
    )

    fun leaveShootCycle() = Sequence(
        //Instant{lights.turnOn()},
        drive.goToCircle(ShootPose.Park.mirroredIf(red), 3.0, 0.06),
        shootAll(),
        //Instant{lights.turnOff()},
        name = "CloseShootCycle"
    )
}

open class Auto(val red: Boolean, val startPose: Pose, val command: Robot.() -> Command) : LinearOpMode(){
    override fun runOpMode() {
        storedRed = Reference(red)
        val robot = Robot(hardwareMap, telemetry)
        val command = robot.command()
        while (opModeInInit()){
            robot.indexTracker.pattern = /*robot.camera.getPattern() ?:*/ robot.indexTracker.pattern
            telemetry.addData("pattern", robot.indexTracker.pattern)
            telemetry.update()
        }

        waitForStart()

        if (!opModeIsActive()){
            return
        }

        robot.drive.localizer.pose = startPose.mirroredIf(storedRed.get())
        robot.drive.startP2PWithTargetPose(ShootPose.Close.mirroredIf(storedRed.get()))
        //robot.camera.sendHeading(startPose.heading)
        runBlocking(command)
    }

}

open class Teleop(val function: Robot.(opmode: LinearOpMode) -> Unit): LinearOpMode(){
    override fun runOpMode() {
        val robot = Robot(hardwareMap, telemetry)
        robot.function(this)
    }
}
