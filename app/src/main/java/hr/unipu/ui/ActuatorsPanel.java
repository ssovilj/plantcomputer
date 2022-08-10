package hr.unipu.ui;

import eu.hansolo.tilesfx.Tile;
import eu.hansolo.tilesfx.TileBuilder;
import hr.unipu.client.MqttClientConnection;
import hr.unipu.event.EventListener;
import hr.unipu.plantcomputer.PlantComputerAction;
import hr.unipu.plantcomputer.PlantComputerCommand;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;

public class ActuatorsPanel extends HBox implements EventListener {

    private static final double TILE_WIDTH  = 150;
    private static final double TILE_HEIGHT = 150;

    private final MqttClientConnection mqttClientConnection;

    private final Tile btGrowLight;
    //private final ToggleButton btGrowLight;

    private PlantComputerAction selectedPlantComputerAction;

    private boolean blockSending = false;

    /**
     * Construct the UI.
     */
    ActuatorsPanel(MqttClientConnection mqttClientConnection) {
        this.mqttClientConnection = mqttClientConnection;

        this.setSpacing(25);

        GridPane gridPane = new GridPane();
        gridPane.setHgap(10);
        gridPane.setVgap(10);
        this.getChildren().add(gridPane);

//        this.btGrowLight = new ToggleButton("Light Switch");
        this.btGrowLight = TileBuilder.create()
                .skinType(Tile.SkinType.SWITCH)
                .prefSize(TILE_WIDTH, TILE_HEIGHT)
                .title("GROW LIGHTS")
                .text("Whatever text")
                //.description("Test")
                .build();
//        this.btLightSwitch.getStyleClass().add("ledButton");
//        this.btStatic.setOnAction(e -> this.setEffect(FoodComputerActions.STATIC));
        gridPane.add(this.btGrowLight, 0, 0);

        btGrowLight.setOnSwitchPressed(e -> System.out.println("Switch pressed."));
        btGrowLight.setOnSwitchReleased(e -> System.out.println("Switch released."));
    }

    @Override
    public void onQueueMessage(PlantComputerCommand plantComputerCommand) {

    }
}
