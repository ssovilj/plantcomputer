package hr.unipu.client;

import hr.unipu.foodcomputer.FoodComputerCommand;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

/**
 * Helper class to add a {@link FoodComputerCommand} to a table with a timestamp.
 */
public class ReceivedMessage {

    private final String timestamp;
    private final FoodComputerCommand foodComputerCommand;

    private final DateTimeFormatter dateFormat = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss");

    public ReceivedMessage(FoodComputerCommand foodComputerCommand) {
        this.timestamp = LocalDateTime.now().format(dateFormat);
        this.foodComputerCommand = foodComputerCommand;
    }

    public String getTimestamp() {
        return timestamp;
    }

    public FoodComputerCommand getFoodComputerCommand() {
        return foodComputerCommand;
    }
}
