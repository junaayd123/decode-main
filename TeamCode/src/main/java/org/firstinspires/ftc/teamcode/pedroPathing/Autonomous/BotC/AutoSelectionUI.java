package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;


import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;
import com.pedropathing.telemetry.SelectableOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.BotC.AutoSelectableOpMode;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

/**
 * We start in the UI thing idk bruhburhbuoerwhboierjbioerw
 * I created a file called AutoSelectableOpMode which is directly related to SelectableOpMode from Pedro Pathing
 *
 * @author Junaayd Shaikh - 23918 Super Sigma Robotics
 * @version 1.0
 */
@Configurable
@Autonomous(name = "Autonomous Selection")
public class AutoSelectionUI extends AutoSelectableOpMode {
    public static Follower follower;

    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();

    public AutoSelectionUI() {
        super("(DO NOT USE YET DO NOT USE YET THIS IS JUST A LAYOUT DO NOT USE) Select an Autonomous", s -> {
            s.folder("Close Red", l -> {
                //l.add("Optimized Close Red", optimizedclosered_webcam::new);
            });
            s.folder("Far Red", l -> {
                //l.add("Optimized Far Red", farredoptimized::new);
            });
            s.folder("Close Blue", l -> {
                //l.add("Optimized Close Blue", optimizedcloseblue_webcam::new);
            });
            s.folder("Far Blue", l -> {
                //l.add("Optimized Far Blue", farblueoptimized::new);
            });
            /*s.folder("Automatic", a -> {
                a.add("Forward Velocity Tuner", ForwardVelocityTuner::new);
                a.add("Lateral Velocity Tuner", LateralVelocityTuner::new);
                a.add("Forward Zero Power Acceleration Tuner", ForwardZeroPowerAccelerationTuner::new);
                a.add("Lateral Zero Power Acceleration Tuner", LateralZeroPowerAccelerationTuner::new);
            });
            s.folder("Manual", p -> {
                p.add("Translational Tuner", TranslationalTuner::new);
                p.add("Heading Tuner", HeadingTuner::new);
                p.add("Drive Tuner", DriveTuner::new);
                p.add("Centripetal Tuner", CentripetalTuner::new);
            });
            s.folder("Tests", p -> {
                p.add("Line", Line::new);
                p.add("Triangle", Triangle::new);
                p.add("Circle", Circle::new);*/
            });
        //});
    }
    public void onSelect() {

    }

}

/*class testOne {

}

class testTwo {

}*/