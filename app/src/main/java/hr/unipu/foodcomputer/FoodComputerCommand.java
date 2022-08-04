package hr.unipu.foodcomputer;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.util.CompactStringObjectMap;

/**
 * FoodComputerCommand as it is exchanged with the Arduino.
 * - creates Mosquitto command string;
 */
public class FoodComputerCommand {
    private String id;
    private String actionName;
    private String actionValue;
    private FoodComputerAction foodComputerAction;

    /**
     * No argument constructor.
     */
    public FoodComputerCommand() {
        this.foodComputerAction = FoodComputerAction.UNDEFINED;
        this.id = foodComputerAction.getId();
        this.actionName = foodComputerAction.getActionName();
        this.actionValue = foodComputerAction.getActionValue();
    }

    /**
     * Constructor with arguments.
     */
    public FoodComputerCommand(FoodComputerAction foodComputerAction) {
        this.foodComputerAction = foodComputerAction;
        this.id = foodComputerAction.getId();
        this.actionName = foodComputerAction.getActionName();
        this.actionValue = foodComputerAction.getActionValue();
    }

    /**
     * Initialize a {@link FoodComputerCommand} from a ":"-separated String as exchanged via Mosquitto.
     * Example: 100                 // id for turning actuator (for light) on
     *          100 : ALPN 1 : on
     *          ALPN 1 : on         // turn actuator (for light) on
     *          SWTM 1 : read       // read sensor (for temperature)
     * @param command {@link String}
     */
    public FoodComputerCommand(String command) {
        FoodComputerAction foodComputerAction = FoodComputerAction.UNDEFINED;
        String[] parts = command.split(":");

        // e.g. 100
        foodComputerAction = parts.length == 1 ? FoodComputerAction.fromId(parts[0].trim()) : FoodComputerAction.UNDEFINED;

        // e.g. ALPN 1 : on
        if (parts.length == 2) {
            boolean containsOnlyNumbers = !parts[0].trim().contains("[a-zA-Z]+");
            if (containsOnlyNumbers) {
                foodComputerAction = FoodComputerAction.UNDEFINED;
            } else {
                this.actionName = parts[0].trim();
                this.actionValue = parts[1].trim();
                foodComputerAction = FoodComputerAction.fromNameAndValue(actionName, actionValue);
            }
        }

        // e.g. 100 : ALPN 1 : on
        if (parts.length == 3) {
            this.id = parts[0].trim();
            this.actionName = parts[1].trim();
            this.actionValue = parts[2].trim();
            foodComputerAction = FoodComputerAction.fromId(id);     // ID is dominant.
        }

        this.id = foodComputerAction.getId();
        this.actionName = foodComputerAction.getActionName();
        this.actionValue = foodComputerAction.getActionValue();

    }


    /**
     * Convert to a ":"-separated string command.
     * e.g. 100:ALPN 1:on
     *
     * @return The command as ":"-separated String
     */
    public String toStringCommand() {
        return this.getId() + ":"
                + this.getActionName() + ":"
                + this.getActionValue();
    }


    /**
     * Convert to JSON String command.
     * e.g. {"id":"100","actionName":"ALPN 1","actionValue":"on"}
     *
     * @return The command as JSON string.
     */
    public String toJsonCommand() {
        ObjectMapper objectMapper = new ObjectMapper();
        String jsonString = "{}";
        try {
            jsonString = objectMapper.writeValueAsString(this);     // FoodComputerCommand object to JSON.
        } catch (JsonProcessingException e) {
            e.printStackTrace();
        }
        return jsonString;
    }


    public String getId() {
        return id;
    }
    public String getActionName() {
        return actionName;
    }
    public String getActionValue() {
        return actionValue;
    }
    public FoodComputerAction getFoodComputerAction() {
        return foodComputerAction;
    }

}
