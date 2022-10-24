package hr.unipu.client;

import hr.unipu.plantcomputer.PlantComputerCommand;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

/**
 * Helper class to add a {@link PlantComputerCommand} to a table with a timestamp.
 */
public class ReceivedMessage {

    private final String timestamp;
    private final PlantComputerCommand plantComputerCommand;

    private final DateTimeFormatter dateFormat = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss");

    public ReceivedMessage(PlantComputerCommand plantComputerCommand) {
        this.timestamp = LocalDateTime.now().format(dateFormat);
        this.plantComputerCommand = plantComputerCommand;
    }

    public String getTimestamp() {
        return timestamp;
    }

    public PlantComputerCommand getPlantComputerCommand() {
        return plantComputerCommand;
    }
}
