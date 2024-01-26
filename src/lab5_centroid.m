javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java;
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.cam.cam_IS.cam.cam_IS.connect();

robot = Robot(myHIDSimplePacketComs);

T_0_Checker = [0, 1, 0, 75;
               1, 0, 0, -100;
               0, 0, -1, 0;
               0, 0, 0, 1];