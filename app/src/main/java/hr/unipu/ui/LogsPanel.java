package hr.unipu.ui;

import hr.unipu.client.ReceivedMessage;
import hr.unipu.event.EventListener;
import hr.unipu.foodcomputer.FoodComputerCommand;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.scene.control.TableCell;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.cell.PropertyValueFactory;

public class LogsPanel extends TableView implements EventListener {

    private ObservableList<ReceivedMessage> list = FXCollections.observableArrayList();

    /**
     * Construct the UI as a {@link TableView}.
     */
    LogsPanel() {

        TableColumn colTimestamp = new TableColumn("Timestamp");
        colTimestamp.setStyle("-fx-alignment: TOP-LEFT;");
        colTimestamp.setMinWidth(150);
        colTimestamp.setCellValueFactory(new PropertyValueFactory<>("timestamp"));


        TableColumn colCommandAction = new TableColumn("Action ID");
        colCommandAction.setStyle("-fx-alignment: TOP-LEFT;");
        colCommandAction.setMinWidth(100);
        colCommandAction.setCellValueFactory(new PropertyValueFactory<>("foodComputerCommand"));
        colCommandAction.setCellFactory(column -> new TableCell<FoodComputerCommand, FoodComputerCommand>() {
            @Override
            protected void updateItem(FoodComputerCommand foodComputerCommand, boolean empty) {
                super.updateItem(foodComputerCommand, empty);
                if (foodComputerCommand == null || empty) {
                    setText(null);
                    setStyle("");
                } else {
                    setText(foodComputerCommand.getId());
                }
            }
        });


        TableColumn colActionName = new TableColumn("Action Name");
        colActionName.setStyle("-fx-alignment: TOP-LEFT;");
        colActionName.setMinWidth(100);
        colActionName.setCellValueFactory(new PropertyValueFactory<>("foodComputerCommand"));
        colActionName.setCellFactory(column -> new TableCell<FoodComputerCommand, FoodComputerCommand>() {
            @Override
            protected void updateItem(FoodComputerCommand item, boolean empty) {
                super.updateItem(item, empty);
                if (item == null || empty) {
                    setText(null);
                    setStyle("");
                } else {
                    setText(String.valueOf(item.getActionName()));
                }
            }
        });


        TableColumn colActionValue = new TableColumn("Action Value");
        colActionValue.setStyle("-fx-alignment: TOP-LEFT;");
        colActionValue.setMinWidth(100);
        colActionValue.setCellValueFactory(new PropertyValueFactory<>("foodComputerCommand"));
        colActionValue.setCellFactory(column -> new TableCell<FoodComputerCommand, FoodComputerCommand>() {
            @Override
            protected void updateItem(FoodComputerCommand item, boolean empty) {
                super.updateItem(item, empty);
                if (item == null || empty) {
                    setText(null);
                    setStyle("");
                } else {
                    setText(String.valueOf(item.getActionValue()));
                }
            }
        });


        TableColumn colDataString = new TableColumn("Data String");
        colDataString.setStyle("-fx-alignment: TOP-CENTER;");
        colDataString.setMinWidth(300);
        colDataString.setCellValueFactory(new PropertyValueFactory<>("foodComputerCommand"));
        colDataString.setCellFactory(column -> new TableCell<FoodComputerCommand, FoodComputerCommand>() {
            @Override
            protected void updateItem(FoodComputerCommand item, boolean empty) {
                super.updateItem(item, empty);
                if (item == null || empty) {
                    setText(null);
                } else {
                    setText(item.toStringCommand());
                }
            }
        });

        this.getColumns().addAll(
                colTimestamp,
                colCommandAction,
                colActionName,
                colActionValue,
                colDataString);

        this.setItems(this.list);
    }

    /**
     * {@link FoodComputerCommand} received from Mosquitto.
     * We put it on top of our internal list so it gets added to the table.
     *
     * @param foodComputerCommand The {@link FoodComputerCommand}
     */
    @Override
    public void onQueueMessage(FoodComputerCommand foodComputerCommand) {
        this.list.add(0, new ReceivedMessage(foodComputerCommand));
    }
}
