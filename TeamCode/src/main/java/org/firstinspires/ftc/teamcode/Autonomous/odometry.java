//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//
//@Config
//public class odometry {
//// xOffset - how sideways from the center of the robot is the X (forward) pod? Left increases
//// yOffset - how far forward from the center of the robot is the Y (Strafe) pod? forward increases
//    setOffsets ( double xOffset, double yOffset);
//    setEncoderResolution (goBILDA_4_BAR_POD)
//    setEncoderDirections(GoBildaPinpointDriver.EncoderDirection xEncoder, EncoderDirection yEncoder);
//    resetPosAndIMU();
//    static double angle=0;
//    static boolean status=false;
//    static double velocity=0;
//    static double posX;
//    static double posY;
//    static double velX;
//    static double velY;
//    static double velAngle;
//    public static void calcodo(){
//        update();
//        getPosition();
//        if(getDeviceStatus()=="READY"){
//            status=true;
//        }
//        else {status=false;}
//
//        angle=getPosition();
//        velocity=getVelocity();
//        posX=getPosX();
//        posY=getPosY();
//        velX=getVelX();
//        velY=getVely();
//        velAngle=getVelH();
//
//    }
//}