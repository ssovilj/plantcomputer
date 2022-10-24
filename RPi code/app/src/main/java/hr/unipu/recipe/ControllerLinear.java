package hr.unipu.recipe;

/**
 * A controller that passes on the measured reading of a sensor as an
 * actuation command.
 */

public class ControllerLinear {

    private Double set_point;
    private String variableName;

    public ControllerLinear() {

        String sub_name = "cmd";
        String state_sub_name = "state";
        String desired_sub_name = "desired";

        /**
         * variable = rospy.get_param("~variable", None)
         *     if variable is not None:
         *         command_pub_name = "{}/commanded".format(variable)
         *         state_pub_name = "{}/raw".format(variable)
         *     else:
         *         command_pub_name = "cmd"
         *         state_pub_name = "state"
         *
         *
         *     command_pub = rospy.Publisher(command_pub_name, Float64, queue_size=10)
         */

    }

    public ControllerLinear(String variableName) {
        this(); // Call no-name constructor.
        this.set_point = null;
        this.variableName = variableName;
    }

    public Double update(Double state) {

        // If setpoint was made null, or was already null, do nothing.
        if (state == null) return null;

        Double res = state;

        return res;
    }

    public void set_point_callback(Double set_point) {
        this.set_point = set_point;
    }
}
