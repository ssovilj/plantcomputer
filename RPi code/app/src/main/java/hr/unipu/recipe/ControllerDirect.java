package hr.unipu.recipe;

/**
 * Direct Controller
 * This ROS node controls the variables which do not have any feedback and the
 * output setting is just set once and maintained.
 * The command and measured topics are simply echo's of the desired topic.
 * This is called from the launch file, for example:
 *
 * <node pkg="openag_brain" type="direct_controller.py" name="light_controller_red_1">
 *   <param name="variable" value="light_intensity_red" type="str"/>
 * </node>
 */
public class ControllerDirect {

    private Double set_point;
    private String variableName;

    public ControllerDirect() {

        String sub_name = "cmd";
        String state_sub_name = "state";
        String desired_sub_name = "desired";

        /**
         * variable = rospy.get_param("~variable", None)
         *     if variable is not None:
         *         command_pub_name = "{}/commanded".format(variable)
         *         state_pub_name = "{}/raw".format(variable)
         *         desired_sub_name = "{}/desired".format(variable)
         *
         *     command_pub = rospy.Publisher(command_pub_name, Float64, queue_size=10)
         *     state_pub = rospy.Publisher(state_pub_name, Float64, queue_size=10)
         */

    }

    public ControllerDirect(String variableName) {
        this(); // Call no-name constructor.
        this.set_point = null;
        this.variableName = variableName;
    }

    public Double update(Double state) {

        // If setpoint was made null, or was already null, do nothing.
        if (state == null) return null;

        Double error = this.set_point - state;

        Double res = error;

        return res;
    }

    public void set_point_callback(Double set_point) {
        this.set_point = set_point;
    }



}
