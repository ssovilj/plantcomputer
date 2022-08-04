package hr.unipu.ui;

import hr.unipu.client.MqttClientConnection;
import hr.unipu.event.EventManager;
import javafx.geometry.Insets;
import javafx.scene.Group;
import javafx.scene.control.Button;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.StackPane;
import javafx.scene.layout.VBox;

public class MenuWindow extends HBox {

    private final Pane pane;
    private final Group home;
    private final Group sensors;
    private final Group actuators;
    private final Group log;

    /**
     * Construct the main UI with the menu buttons.
     */
    public MenuWindow(EventManager eventManager, MqttClientConnection mqttClientConnection) {
        this.setSpacing(25);
        this.getStylesheets().add("styles/style.css");
        this.getStyleClass().add("bg");

        // Adding to Hbox a left pane: VBox main menu 'menuButtons'.
        this.getChildren().add(this.getMainMenu());

        // Adding to Hbox a right pane: StackPane (HomePanel/SensorsPanel/ActuatorsPanel/LogsPanel)
        this.pane = new StackPane();
        this.getChildren().add(this.pane);

        // Home/Camera group pane.
        this.home = new Group();

        // Sensors group panel.
        SensorsPanel sensorsPanel = new SensorsPanel(mqttClientConnection);
        eventManager.addListener(sensorsPanel);
        this.sensors = new Group(sensorsPanel);

        // Actuators group panel.
        ActuatorsPanel actuatorPanel = new ActuatorsPanel(mqttClientConnection);
        eventManager.addListener(actuatorPanel);
        actuatorPanel.prefWidthProperty().bind(this.widthProperty());
        actuatorPanel.prefHeightProperty().bind(this.heightProperty());
        this.actuators = new Group(actuatorPanel);

        // Message list group panel.
        LogsPanel logsPanel = new LogsPanel();
        eventManager.addListener(logsPanel);
        logsPanel.prefWidthProperty().bind(this.widthProperty());
        logsPanel.prefHeightProperty().bind(this.heightProperty());
        this.log = new Group(logsPanel);

        this.show(this.actuators);
    }

    /**
     * Builds the main menu button bar.
     *
     * @return {@link VBox}
     */
    private VBox getMainMenu() {

        final VBox menuButtons = new VBox();
        menuButtons.setPadding(new Insets(5, 5, 5, 5));
        menuButtons.setSpacing(5);

        final Button btCamera = new Button("Camera");
        btCamera.getStyleClass().add("menuButton");
        btCamera.setOnAction(e -> this.show(this.home));
        btCamera.setMaxSize(Double.MAX_VALUE, Double.MAX_VALUE);
        menuButtons.getChildren().add(btCamera);

        final Button btSensors = new Button("Sensors");
        btSensors.getStyleClass().add("menuButton");
        btSensors.setOnAction(e -> this.show(this.sensors));
        btSensors.setMaxSize(Double.MAX_VALUE, Double.MAX_VALUE);
        menuButtons.getChildren().add(btSensors);

        final Button btActuators = new Button("Actuators");
        btActuators.getStyleClass().add("menuButton");
        btActuators.setOnAction(e -> this.show(this.actuators));
        btActuators.setMaxSize(Double.MAX_VALUE, Double.MAX_VALUE);
        menuButtons.getChildren().add(btActuators);

        final Button btLog = new Button("Logs");
        btLog.getStyleClass().add("menuButton");
        btLog.setOnAction(e -> this.show(this.log));
        btLog.setMaxSize(Double.MAX_VALUE, Double.MAX_VALUE);
        menuButtons.getChildren().add(btLog);

        return menuButtons;
    }

    /**
     * Switch to the given screen.
     *
     * @param group The group node to be shown
     */
    private void show(Group group) {
        this.pane.getChildren().clear();
        this.pane.getChildren().add(group);
    }
}
