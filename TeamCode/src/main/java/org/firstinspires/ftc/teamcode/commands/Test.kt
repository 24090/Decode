import com.rathippo.commandviewer.CommandViewer
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.ForeverCommand
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.runBlocking


fun main() {
    println(CommandViewer.commandLog)
    val command = ForeverCommand {
        Sequence(
            Sleep(10.0, "10 Second Sleep"),
            Sleep(3.0, "3 Second Sleep")
        )
    }
    runBlocking(command)
}
