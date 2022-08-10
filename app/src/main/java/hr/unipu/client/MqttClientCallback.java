package hr.unipu.client;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import hr.unipu.event.EventManager;
import hr.unipu.plantcomputer.PlantComputerCommand;
import javafx.application.Platform;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttMessage;

/**
 * Implementation of MqttCallback interface methods: connectionLost(), deliveryComplete(), messageArrived().
 */
public class MqttClientCallback implements MqttCallback {

    final EventManager eventManager;


    public MqttClientCallback(EventManager eventManager) {
        this.eventManager = eventManager;
    }


    @Override
    public void connectionLost(Throwable throwable) {
        System.out.println("Connection to MQTT broker lost!");
    }


    @Override
    public void messageArrived(String mqttTopic, MqttMessage mqttMessage) {
        String jsonMessage = new String(mqttMessage.getPayload());

        if (mqttTopic.equals("plantComputerCommand")) {
            System.out.println("Filtered MQTT topic: \"" + mqttTopic + "\".");
            System.out.println("Message received:\n\t" + jsonMessage);
            Platform.runLater(() -> {
                System.out.println("Forwarding MQTT message (of actuators) to all listeners.");
                parseJsonMessage2Commands(jsonMessage);

            });

        } else if (mqttTopic.equals("plantComputerState")) {
            System.out.println("Filtered MQTT topic: \"" + mqttTopic + "\".");
            System.out.println("Message received:\n\t" + jsonMessage);
            Platform.runLater(() -> {
                System.out.println("Forwarding MQTT message (from sensors) to all listeners.");
                parseJsonMessage2Readings(jsonMessage);
            });

        }


    }

    /**
     * Parse jsonMessage to commands.
     * @param jsonMessage
     */
    private void parseJsonMessage2Commands(String jsonMessage) {
        ObjectMapper objectMapper = new ObjectMapper();
        //TypeFactory typeFactory = objectMapper.getTypeFactory();
        try {
            //List<FoodComputerCommand> parsedListActuatorCommands = objectMapper.readValue(jsonMessage, typeFactory.constructCollectionType(List.class, FoodComputerCommand.class));
            PlantComputerCommand[] parsedListActuatorCommands = objectMapper.readValue(jsonMessage, PlantComputerCommand[].class);
            for (PlantComputerCommand parsedListActuatorCommand : parsedListActuatorCommands) {
                //System.out.println(parsedListActuatorCommand.toStringCommand());
                eventManager.sendEvent(parsedListActuatorCommand);
            }
        } catch (JsonProcessingException e) {
            e.printStackTrace();
        }
    }


    /**
     * Parse jsonMessage to sensors' readings.
     * @param jsonMessage
     */
    private void parseJsonMessage2Readings(String jsonMessage) {
        ObjectMapper objectMapper = new ObjectMapper();
        try {
            PlantComputerCommand[] parsedListSensorReadings = objectMapper.readValue(jsonMessage, PlantComputerCommand[].class);
            for (PlantComputerCommand parsedListActuatorReading : parsedListSensorReadings) {
                //System.out.println(parsedListActuatorReading.toStringCommand());
                eventManager.sendEvent(parsedListActuatorReading);
            }
        } catch (JsonProcessingException e) {
            e.printStackTrace();
        }
    }


    @Override
    public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {
        System.out.println("Delivery complete.");
    }

}