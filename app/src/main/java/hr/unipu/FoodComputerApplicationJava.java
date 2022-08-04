package hr.unipu;

import hr.unipu.client.MqttClientConnection;
import hr.unipu.event.EventManager;
import hr.unipu.server.WebHandler;
import hr.unipu.ui.UiWindow;
import io.undertow.Undertow;
import javafx.application.Application;
import javafx.scene.Scene;
import javafx.stage.Stage;

/**
 * Food Computer Controller JavaFX class.
 * - 'Application' class provides lifecycle functions (initializing, launching, starting, stopping) during runtime.
 * - Mechanism to launch JavaFX GUI components separate from the 'main' thread.
 */
public class FoodComputerApplicationJava extends Application {


    /**
     * JavaFX app entry point. (main thread)
     * @param args Command line arguments.
     */
    public static void main(String[] args) {
        // On the 'main thread'.

        // Later, to access any (named or raw) arguments invoke the 'getParameters()' method
        // on the 'Application' class.
        Application.launch(args);
    }


    /**
     * start() method with auto-generated primary stage. (application thread)
     * @param stage JavaFX auto-generated primary stage.
     *
     * Initialize the {@link EventManager} and {@link MqttClientConnection}, start the webserver and show the UI.
     */
    @Override
    public void start(Stage stage) {
        EventManager eventManager = new EventManager();

        // 5.17 (Vg), 0.30 (Zg), 3.2 (Ethernet direct)
        // Fonović MQTT public IP: serv.ovh.dfo.ninja; topics: "foodComputerCommand" i "foodComputerState"
        //QueueClient queueClient = new QueueClient(eventManager, "192.168.0.21", "foodComputerCommand");
        MqttClientConnection mqttClientConnection = new MqttClientConnection( "serv.ovh.dfo.ninja", eventManager);
        mqttClientConnection.subscribe("foodComputerCommand");
        mqttClientConnection.subscribe("foodComputerState");

        try {
            Undertow server = Undertow.builder()
                    .addHttpListener(8080, "localhost")
                    .setHandler(new WebHandler(mqttClientConnection))
                    .build();
            server.start();
        } catch (Exception e) {
            e.printStackTrace();
        }

        //var scene = new Scene(new MenuWindow(eventManager, queueClient), 1024, 600);
        var scene = new Scene(new UiWindow(eventManager, mqttClientConnection), 1280, 800);

        stage.setScene(scene);
        //stage.setFullScreen(true);
        stage.show();
    }
}