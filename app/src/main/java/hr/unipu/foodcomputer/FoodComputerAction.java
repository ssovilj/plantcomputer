package hr.unipu.foodcomputer;

/**
 * The list of available Food Computer actions.
 * By using enums with variables, we can define here which UI-elements must be enabled when an action is selected.
 */
public enum FoodComputerAction {

    // TODO Delete all comments from previous code or translate them to English.

    /*
    “{"GTYP":"Stream","SWPH 1":6.9,"SWTM 1":25.4,"SWEC 1":2.6,"SLIN 1":33,"SLPA
      1":0.73,"SATM 1":24.7,"SAHU 1":31.3,"SACO 1":400,"SATM 2":25.2,"SAHU 2":39.0,
      1":1,"AAHE 1":0,"AAHU 1":0,"AAVE 1":1,"AACR 1":1,"ALPN 1":0, "GEND":0}”

      "GTYP":"Stream" – Stream poruka
      "SWPH 1":6.9 – 1.  Senzor pH vode (Water PH) vraća vrijednost  6.9
      "SWTM 1":25.4 – 1. Senzor temperature vode (Water Temperature) vraća vrijednost od 25.4 stupnjeva celzijusa
      "SWEC 1":2.6 - 1. Senzor električne provodljivosti (Water Electrical Conductivity) vraća vrijednost od  2.6 mS/cm
      "SLIN 1":33 - 1. Sensor intenziteta svjetlosti (Light Intensity) vraća vrijednost od 33 lux
      "SLPA 1":0.73 - 1. Senzor svjetlosti par (Light PAr (Photosynthetically Active Radiation)) vraća vrijednost od  0.73 umol/(s*m^2)
      "SATM 1":24.7 - 1. Senzor temperature zraka (Air TeMperature) vraća vrijednost od 24.7 stupnjeva celzijusa
      "SAHU 1":31.3 - 1. Senzor vlažnosti zraka (Air Humidity) vraća vrijednost od 31.3 % relatvne vlažnosti
      "SACO 1":400 - 1. CO2 Senzor (Air CO2) vraća vrijednost od 400 ppm (parts per million)
      "SATM 2":25.2 – 2. Senzor temperature zraka (Air Temperature) vraća vrijednost od 25.2 stupnjeva celzijusa
      "SAHU 2":39.0 - 2. Senzor vlažnosti zraka (Air Humidity) vraća vrijednost od 39.0 % relatvne vlažnosti
      "AAHE 1":0 - 1. Aktuator grijača  (Air Heater) vraća 0 (off)
      "AAHU 1":0 - 1. Aktuator ovlaživača (Air Humidifier) vraća 0 (off)
      "AAVE 1":1 - 1. Aktuator ventilacije(Air Vent) vraća 1 (on)
      "AACR 1":1 - 1. Aktuator cirkulacije zraka (Air CiRculation) vraća 1 (on)
      "ALPN 1":0 - 1. Aktuator svjetla (Light PaNel) vraća 0 (off)
      "GEND":0 – Poruka za kraj
    */

    UNDEFINED("000","undefined_key", "undefined_value"),
    TEST_ALL("010","test","all"),
    TEST_ACTUATORS("010","test","actuators"),
    TEST_SENSORS("050","test","sensors"),

    /**
     * Actuators' (default) states:
     */
    LIGHT_STATE( "100","ALPN 1", "off"),
    HUMIDIFIER_STATE( "110","ALHU 1", "off"),
    COOLING_FAN_STATE( "120","ALVE 1", "off"),
    CHAMBER_FAN_STATE( "130","ALCR 1", "off"),
    HEATER_TURN_STATE( "140","ALHE 1", "off"),
    WATER_PUMP_STATE( "150","ALWP 1", "off"),

    /**
     * Actuators' actions:
     */
    LIGHT_TURN_ON( "101","ALPN 1", "on"),
    LIGHT_TURN_OFF( "102","ALPN 1", "off"),
    LIGHT_TOGGLE( "103","ALPN 1", "toggle"),    // Toggle for fun.

    LIGHT_RED_TURN_ON( "201","ALPR 1", "on"),       // light_intensity_red
    LIGHT_RED_TURN_OFF( "202","ALPR 1", "off"),
    LIGHT_BLUE_TURN_ON( "211","ALPB 1", "on"),      // light_intensity_blue
    LIGHT_BLUE_TURN_OFF( "212","ALPB 1", "off"),
    LIGHT_WHITE_TURN_ON( "221","ALPW 1", "on"),     // light_intensity_white
    LIGHT_WHITE_TURN_OFF( "222","ALPW 1", "off"),

    HUMIDIFIER_TURN_ON( "111","ALHU 1", "on"),
    HUMIDIFIER_TURN_OFF( "112","ALHU 1", "off"),
    CHILLER_FAN_TURN_ON( "121","ALVE 1", "on"),     // chiller_fan_1
    CHILLER_FAN_TURN_OFF( "122","ALVE 1", "off"),
    CHAMBER_FAN_TURN_ON( "131","ALCR 1", "on"),     // chamber_fan_1
    CHAMBER_FAN_TURN_OFF( "132","ALCR 1", "off"),
    HEATER_TURN_ON( "141","ALHE 1", "on"),          // heater_core_1_1 | heater_core_2_1
    HEATER_TURN_OFF( "142","ALHE 1", "off"),
    WATER_PUMP_TURN_ON( "151","ALWP 1", "on"),
    WATER_PUMP_TURN_OFF( "152","ALWP 1", "off"),    // ??? pump_5_water_1

    WATER_AERATION_PUMP_TURN_ON( "231","ALWA 1", "on"),         // water_aeration_pump_1
    WATER_AERATION_PUMP_TURN_OFF( "232","ALWA 1", "off"),
    WATER_CIRCULATION_PUMP_TURN_ON( "231","ALWC 1", "on"),      // water_circulation_pump_1
    WATER_CIRCULATION_PUMP_TURN_OFF( "231","ALWC 1", "off"),
    CHILLER_PUMP_TURN_ON("241", "ALCP 1", "on"),                // chiller_pump_1
    CHILLER_PUMP_TURN_OFF("242", "ALCP 1", "off"),
    CHILLER_COMPRESSOR_TURN_ON("251", "ALCC 1", "on"),          // chiller_compressor_1
    CHILLER_COMPRESSOR_TURN_OFF("252", "ALCC 1", "off"),

    AIR_FLUSH_TURN_ON("261", "ALAF 1", "on"),       // ??? air_flush_1
    AIR_FLUSH_TURN_OFF("261", "ALAF 1", "on"),

    PUMP_PH_UP_ON("161","APHU 1","on"),     // pump_3_ph_up_1
    PUMP_PH_UP_OFF("162","APHU 1","off"),
    PUMP_PH_DOWN_ON("171","APHD 1","on"),   // pump_4_ph_down_1
    PUMP_PH_DOWN_OFF("172","APHD 1","on"),

    PUMP_NUTRIENT_A_ON("181", "ANUA 1", "on"),      // "pump_1_nutrient_a_1"
    PUMP_NUTRIENT_A_OFF("182", "ANUA 1", "off"),
    PUMP_NUTRIENT_B_ON("191", "ANUB 1", "on"),      // "pump_2_nutrient_b_1"
    PUMP_NUTRIENT_B_OFF("192", "ANUB 1", "off"),


    /**
     * Actuators' read actions:
     */
    LIGHT_STATE_READ( "105","ALPN 1", "read"),
    HUMIDIFIER_STATE_READ( "115","ALHU 1", "read"),
    COOLING_FAN_STATE_READ( "125","ALVE 1", "read"),
    CHAMBER_FAN_STATE_READ( "135","ALCR 1", "read"),
    HEATER_TURN_STATE_READ( "145","ALHE 1", "read"),
    WATER_PUMP_STATE_READ( "155","ALWP 1", "read"),


    /**
     * Sensors' (default) readings:
     */
    TEMPERATURE_AIR_STATE( "500", "SATM 1", "27.0"),        // "air_temperature"
    HUMIDITY_AIR_STATE("510", "SAHU 1", "60.0"),            // "air_humidity"
    CONDUCTIVITY_WATER_STATE("520", "SWEC 1", "1.5"),       // "water_electrical_conductivity"
    PH_WATER_STATE("530", "SWPH 1", "6.4"),                 // "water_potential_hydrogen"
    TEMPERATURE_WATER_STATE("540", "SWTM 1", "18.33"),      // "water_temperature"
    LIGHT_INTENSITY_STATE("550", "SLIN 1", "15000.0"),

    AIR_CO_STATE("560", "SACO 1", "400"),                   // "air_carbon_dioxide"
    WATER_LEVEL_LOW_STATE("570", "SWLL 1", "0"),            // "water_level_low"
    WATER_LEVEL_HIGH_STATE("570", "SWLH 1", "0"),           // "water_level_high"

    /**
     * Sensors' read actions:
     */
    TEMPERATURE_AIR_STATE_READ( "505", "SATM 1", "read"),
    HUMIDITY_AIR_STATE_READ("515", "SAHU 1", "read"),
    CONDUCTIVITY_WATER_STATE_READ("525", "SWEC 1", "read"),
    PH_WATER_STATE_READ("535", "SWPH 1", "read"),
    TEMPERATURE_WATER_STATE_READ("545", "SWTM 1", "read"),
    LIGHT_INTENSITY_STATE_READ("555", "SLIN 1", "read"),

    AIR_CO_STATE_READ("565", "SACO 1", "read"),                   // "air_carbon_dioxide"
    WATER_LEVEL_LOW_STATE_READ("575", "SWLL 1", "read"),            // "water_level_low"
    WATER_LEVEL_HIGH_STATE_READ("575", "SWLH 1", "read");           // "water_level_high"


    private final String id;
    private final String actionName;
    private String actionValue;

    FoodComputerAction(String id, String actionName, String actionValue) {
        this.id = id;
        this.actionName = actionName;
        this.actionValue = actionValue;
    }

    public static FoodComputerAction fromId(String id) {
        for (FoodComputerAction foodComputerAction : FoodComputerAction.values()) {
            if (foodComputerAction.id.equals(id)) {
                return foodComputerAction;
            }
        }
        return UNDEFINED;
    }

    public static FoodComputerAction fromNameAndValue(String actionName, String actionValue) {
        for (FoodComputerAction foodComputerAction : FoodComputerAction.values()) {
            if (foodComputerAction.actionName.equals(actionName) &&
                    foodComputerAction.actionValue.equals(actionValue)) {
                return foodComputerAction;
            }
        }
        return UNDEFINED;
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
    public FoodComputerAction setActionValue(String actionValue) {
        this.actionValue = actionValue;
        return this;
    }

}
