import com.rathippo.commandviewer.CommandViewer
import org.firstinspires.ftc.teamcode.commands.Command
import org.firstinspires.ftc.teamcode.commands.CommandResult
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.runBlocking


fun main() {
    println(CommandViewer.commandLog)
    val command = Forever({
        Sequence(
            Sleep(10.0, "10 Second Sleep"),
            Instant({ println("Hello") }),
            Sleep(3.0, "3 Second Sleep")
        )
    })
    runBlocking(command)
}
