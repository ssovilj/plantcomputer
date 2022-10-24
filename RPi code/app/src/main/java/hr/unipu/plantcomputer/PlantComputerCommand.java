package hr.unipu.plantcomputer;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

/**
 * FoodComputerCommand as it is exchanged with the Arduino.
 * - creates Mosquitto command string;
 */
public class PlantComputerCommand {
    private String id;
    private String actionName;
    private String actionValue;
    private PlantComputerAction plantComputerAction;

    /**
     * No argument constructor.
     */
    public PlantComputerCommand() {
        this.plantComputerAction = PlantComputerAction.UNDEFINED;
        this.id = plantComputerAction.getId();
        this.actionName = plantComputerAction.getActionName();
        this.actionValue = plantComputerAction.getActionValue();
    }

    /**
     * Constructor with arguments.
     */
    public PlantComputerCommand(PlantComputerAction plantComputerAction) {
        this.plantComputerAction = plantComputerAction;
        this.id = plantComputerAction.getId();
        this.actionName = plantComputerAction.getActionName();
        this.actionValue = plantComputerAction.getActionValue();
    }

    /**
     * Initialize a {@link PlantComputerCommand} from a ":"-separated String as exchanged via Mosquitto.
     * Example: 100                 // id for turning actuator (for light) on
     *          100 : ALPN 1 : on
     *          ALPN 1 : on         // turn actuator (for light) on
     *          SWTM 1 : read       // read sensor (for temperature)
     * @param command {@link String}
     */
    public PlantComputerCommand(String command) {
        PlantComputerAction plantComputerAction = PlantComputerAction.UNDEFINED;
        String[] parts = command.split(":");

        // e.g. 100
        plantComputerAction = parts.length == 1 ? PlantComputerAction.fromId(parts[0].trim()) : PlantComputerAction.UNDEFINED;

        // e.g. ALPN 1 : on
        if (parts.length == 2) {
            boolean containsOnlyNumbers = !parts[0].trim().contains("[a-zA-Z]+");
            if (containsOnlyNumbers) {
                plantComputerAction = PlantComputerAction.UNDEFINED;
            } else {
                this.actionName = parts[0].trim();
                this.actionValue = parts[1].trim();
                plantComputerAction = PlantComputerAction.fromNameAndValue(actionName, actionValue);
            }
        }

        // e.g. 100 : ALPN 1 : on
        if (parts.length == 3) {
            this.id = parts[0].trim();
            this.actionName = parts[1].trim();
            this.actionValue = parts[2].trim();
            plantComputerAction = PlantComputerAction.fromId(id);     // ID is dominant.
        }

        this.id = plantComputerAction.getId();
        this.actionName = plantComputerAction.getActionName();
        this.actionValue = plantComputerAction.getActionValue();

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
    public PlantComputerAction getPlantComputerAction() {
        return plantComputerAction;
    }

}
