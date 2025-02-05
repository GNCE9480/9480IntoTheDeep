package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.0254;
        TwoWheelConstants.strafeTicksToInches = -.00895;
        TwoWheelConstants.forwardY = 4.5;
        TwoWheelConstants.strafeX = -9.5;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "left_back_drive";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "right_back_drive";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
    }
}




