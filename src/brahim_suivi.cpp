// openni_tracker.cpp
//#include <geometry_msgs/>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <kinect_suivi_av/msg0.h>
#include <hackbot_ROS_pkg/msg0.h>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

using std::string;
std::string brahim_ss;
ros::Publisher brahim_sss,terminal_Publisher;
float vitesse_v=0,vitesse_w=0,X,Y,Z,qX,qY,qZ,W;







xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("New User %d", nId);

	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("Lost user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	ROS_INFO("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		ROS_INFO("Calibration complete, start tracking user %d", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else {
		ROS_INFO("Calibration failed for user %d", nId);
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
    ROS_INFO("Pose %s detected for user %d", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}



/*void publishTransform1(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id,
                         string const& child_frame_id)
{
  static tf::TransformBroadcaster br;




  XnSkeletonJointPosition joint_position;
  g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
  double x1 = -joint_position.position.X / 1000.0;
  double y1 = joint_position.position.Y / 1000.0;
  double z1 = joint_position.position.Z / 1000.0;

  XnSkeletonJointOrientation joint_orientation;
  g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

  XnFloat* m = joint_orientation.orientation.elements;
  KDL::Rotation rotation(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
  double qx1, qy1, qz1, qw1;
  rotation.GetQuaternion(qx1, qy1, qz1, qw1);
X=x1;Y=y1;Z=z1;qX=qx1; qY=qy1; qZ=qz1; W=qw1;

  /*char child_frame_no[128];
  //snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);
/
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x1,y1, z1));
  transform.setRotation(tf::Quaternion(qx1,-qy1,-qz1, qw1));

  // #4994
  tf::Transform change_frame;
  change_frame.setOrigin(tf::Vector3(0, 0, 0));
  tf::Quaternion frame_rotation;
  frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
  change_frame.setRotation(frame_rotation);

  transform = change_frame * transform;

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "brahim_s1", brahim_ss));
  /*if (user == default_user)
  {
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "brahim_s1", brahim_ss.c_str()));
  }/
}


*/




void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id ) {
    static tf::TransformBroadcaster br;
ros::NodeHandle hh,h1;
 
   XnSkeletonJointPosition joint_position;
    
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2],
    					   m[3], m[4], m[5],
    					   m[6], m[7], m[8]);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);
		terminal_Publisher= hh.advertise<kinect_suivi_av::msg0>("commande_position", 1000);
	kinect_suivi_av::msg0 position;
//brahim_sss= h1.advertise<hackbot_ROS_pkg::msg0>("commande_velocity", 10);
	//old_openni_tracker::msg0 position;
//hackbot_ROS_pkg::msg0 vites;

		//geometry_msgs::Quaternion orientation;
		position.h=child_frame_id;
//if (child_frame_id=="la_tete")
//{
		position.x1 = x;
		position.y1 = y;
		position.z1 = z;

		position.qx1 = qx;
		position.qy1 = qy;
		position.qz1 = qz;
		position.qw1 = qw;
/*}else
{ if (child_frame_id=="left_shoulder")
{
		position.x2 = x;
		position.y2 = y;
		position.z2 = z;

		position.qx2 = qx;
		position.qy2 = qy;
		position.qz2 = qz;
		position.qw2 = qw;
}}/*else { if (child_frame_id=="right_shoulder")
{
		position.x3 = x;
		position.y3 = y;
		position.z3 = z;

		position.qx3 = qx;
		position.qy3 = qy;
		position.qz3 = qz;
		position.qw3 = qw;
}else 
{
		position.x4 = x;
		position.y4 = y;
		position.z4 = z;

		position.qx4 = qx;
		position.qy4 = qy;
		position.qz4 = qz;
		position.qw4 = qw;
}
}}*/
terminal_Publisher.publish(position);
//brahim_sss.publish(vites);

		/*old_openni_tracker.name.push_back(child_frame_id);
		old_openni_tracker.position.push_back(position);
		old_openni_tracker.orientation.push_back(orientation);
		old_openni_tracker.confidence.push_back(joint_position.fConfidence);*/




    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    // #4994
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);







    transform = change_frame * transform;
//x=X;y=Y;qw=W;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
//printf("x=%f\n y=%f\n z=%f\n qx=%f\n qy=%f\n qz=%f\n qw=%f\n ",x,y,z,qx,qy,qw);
//command (X,Y,W);


//static tf::TransformBroadcaster brr;
//while(ros::ok())
//{





//}

/*tf::Transform vitesse;
change_frame.setOrigin(tf::Vector3(vitesse_v, vitesse_v, 0.0));

tf::Quaternion brahim;

brahim.setRPY(x, z, 0);
   transform.setRotation(brahim);
   brr.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", brahim_ss));
*/

//printf("v=%f\n w=%f\n",vitesse_v,vitesse_w);
}

void publishTransform1(XnUserID const& user, XnSkeletonJoint const& joint, XnSkeletonJoint const& joint1, XnSkeletonJoint const& joint2,  XnSkeletonJoint const& joint3,string const& frame_id, string const& child_frame_id ) {
    static tf::TransformBroadcaster br;
ros::NodeHandle hh,h1;
 
   XnSkeletonJointPosition joint_position;
    
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2],
    					   m[3], m[4], m[5],
    					   m[6], m[7], m[8]);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

   XnSkeletonJointPosition joint1_position;
    
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint1, joint1_position);
    double x1 = -joint1_position.position.X / 1000.0;
    double y1 = joint1_position.position.Y / 1000.0;
    double z1 = joint1_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint1_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint1, joint1_orientation);

    XnFloat* m1 = joint1_orientation.orientation.elements;
    KDL::Rotation rotation1(m1[0], m1[1], m1[2],
    					   m1[3], m1[4], m1[5],
    					   m1[6], m1[7], m1[8]);
    double qx1, qy1, qz1, qw1;
    rotation1.GetQuaternion(qx1, qy1, qz1, qw1);

//****************************************///
XnSkeletonJointPosition joint2_position;
    
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint2, joint2_position);
    double x2 = -joint2_position.position.X / 1000.0;
    double y2 = joint2_position.position.Y / 1000.0;
    double z2 = joint2_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint2_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint2, joint2_orientation);

    XnFloat* m2 = joint2_orientation.orientation.elements;
    KDL::Rotation rotation2(m2[0], m2[1], m2[2],
    					   m2[3], m2[4], m2[5],
    					   m2[6], m2[7], m2[8]);
    double qx2, qy2, qz2, qw2;
    rotation2.GetQuaternion(qx2, qy2, qz2, qw2);
//**************************************///
XnSkeletonJointPosition joint3_position;
    
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint3, joint3_position);
    double x3 = -joint2_position.position.X / 1000.0;
    double y3 = joint2_position.position.Y / 1000.0;
    double z3 = joint2_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint3_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint3, joint3_orientation);

    XnFloat* m3 = joint3_orientation.orientation.elements;
    KDL::Rotation rotation3(m3[0], m3[1], m3[2],
    					   m3[3], m3[4], m3[5],
    					   m3[6], m3[7], m3[8]);
    double qx3, qy3, qz3, qw3;
    rotation3.GetQuaternion(qx3, qy3, qz3, qw3);



		terminal_Publisher= h1.advertise<kinect_suivi_av::msg0>("commande_position", 1);
	kinect_suivi_av::msg0 position;
//brahim_sss= h1.advertise<hackbot_ROS_pkg::msg0>("commande_velocity", 10);
	//old_openni_tracker::msg0 position;
//hackbot_ROS_pkg::msg0 vites;

		//geometry_msgs::Quaternion orientation;
		//position.h=child_frame_id;
position.x3 = x;
		position.y3 = y;
		position.z3 = z;

		position.qx3 = qx;
		position.qy3 = qy;
		position.qz3 = qz;
		position.qw3 = qw;
		position.x2 = x1;
		position.y2 = y1;
		position.z2 = z1;

		position.qx2 = qx1;
		position.qy2 = qy1;
		position.qz2 = qz1;
		position.qw2 = qw1;
position.x1 = x2;
		position.y1 = y2;
		position.z1 = z2;

		position.qx1= qx2;
		position.qy1 = qy2;
		position.qz1 = qz2;
		position.qw1 = qw2;
position.x4 = x3;
		position.y4 = y3;
		position.z4 = z3;

		position.qx4 = qx3;
		position.qy4 = qy3;
		position.qz4 = qz3;
		position.qw4 = qw3;

terminal_Publisher.publish(position);
//brahim_sss.publish(vites);

		/*old_openni_tracker.name.push_back(child_frame_id);

		old_openni_tracker.position.push_back(position);

		old_openni_tracker.orientation.push_back(orientation);

		old_openni_tracker.confidence.push_back(joint_position.fConfidence);*/




    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    // #4994
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);







    transform = change_frame * transform;
//x=X;y=Y;qw=W;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
//printf("x=%f\n y=%f\n z=%f\n qx=%f\n qy=%f\n qz=%f\n qw=%f\n ",x,y,z,qx,qy,qw);
//command (X,Y,W);


//static tf::TransformBroadcaster brr;
//while(ros::ok())
//{





//}

/*tf::Transform vitesse;
change_frame.setOrigin(tf::Vector3(vitesse_v, vitesse_v, 0.0));

tf::Quaternion brahim;


brahim.setRPY(x, z, 0);

   transform.setRotation(brahim);
   brr.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", brahim_ss));

*/

//printf("v=%f\n w=%f\n",vitesse_v,vitesse_w);
}
//terminal_Publisher.publish(position);

void publishTransforms(const std::string& frame_id) 
{
//ros::Subscriber sub = node.subscribe(brahim_ss+"/Skeleton", 10, &publishTransform1);
    XnUserID users[1];
//double x,y,z,qx,qy,qz,w,
    XnUInt16 users_count = 1;
    g_UserGenerator.GetUsers(users, users_count);
//float X,Y,Z,qx,qy,qz,w;
    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;


      //publishTransform(user, XN_SKEL_HEAD,           frame_id, "la_tete");
//publishTransform1(user, XN_SKEL_HEAD,           frame_id, "la tete");
//printf("x=%f\n",X);
        //publishTransform(user, XN_SKEL_NECK,           frame_id, "neck");
         //publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso");

        publishTransform1(user, XN_SKEL_LEFT_SHOULDER,XN_SKEL_TORSO,XN_SKEL_HEAD,XN_SKEL_RIGHT_SHOULDER,  frame_id, "left_shoulder");
     //    publishTransform(user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow");
     //    publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand");

        //publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder");
 //        publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow");
 //        publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand");
 //
 //        publishTransform(user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip");
 //        publishTransform(user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee");
 //        publishTransform(user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot");

 //        publishTransform(user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip");
 //        publishTransform(user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee");
 //        publishTransform(user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot");
    }
}

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}

int main(int argc, char **argv) {
    ros::init(argc, argv, "brahim_suivi1");
    ros::NodeHandle nh;
//ros::NodeHandle hh1;
//terminal_Publisher= hh1.advertise<old_openni_tracker::msg0>("commande_position", 10);
//		old_openni_tracker::msg0 position;	
//terminal_Publisher.publish(position);
ros::NodeHandle node("~");


  //ros::Subscriber sub = node.subscribe(brahim_ss+"/Skeleton", 10, &publishTransform1);

    string configFilename = ros::package::getPath("openni_tracker") + "/openni_tracker.xml";
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
    CHECK_RC(nRetVal, "InitFromXml");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		ROS_INFO("Supplied user generator doesn't support skeleton");
		return 1;
	}

    XnCallbackHandle hUserCallbacks;
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

	XnCallbackHandle hCalibrationCallbacks;
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
			ROS_INFO("Pose required, but not supported");
			return 1;
		}

		XnCallbackHandle hPoseCallbacks;
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	ros::Rate r(30);

       

        ros::NodeHandle pnh("~");
        string frame_id("openni_depth_frame");
        pnh.getParam("camera_frame_id", frame_id);

//ros::NodeHandle nh;

//if (argc != 2){
//ROS_ERROR("need turtle name as argument"); 
//return -1;
//}
  //brahim_ss = argv[1];
//ros::NodeHandle node;            
//ros::Subscriber sub = node.subscribe(brahim_ss+"/openni_tracker.xml", 10, &publishTransform);    
	while (ros::ok()) {
		g_Context.WaitAndUpdateAll();
		publishTransforms(frame_id);
		r.sleep();
	}

	g_Context.Shutdown();
	return 0;

}


/*void pubcomande()
{
pub_Publisher= pub_NodeHandle.advertise<old_openni_tracker::msg0>("commande_velocity", 4);
/*lms_Publisher= lms_NodeHandle.advertise<LMS_ROS_pkg::lms_commande>("commande_LMS", 1);*/
//command();
//} 

/*void command(float x,float y,float qw)
{


if (x==2)
{
if (y<0)
{
vitesse_v=0;
vitesse_w=-qw;
}
else if (y>0)
{
vitesse_v=0;
vitesse_w=qw;
}
else {
vitesse_v=0;
vitesse_w=0;
}}
else if (x>2.1 && x<3.5)
{
if (y<0)
{
vitesse_v=2;
vitesse_w=-qw;
}
else if (y>0)
{
vitesse_v=2;
vitesse_w=qw;
}
else {
vitesse_v=2;
vitesse_w=0;
}
}
else if (x>3.5)
{
vitesse_v=0;
vitesse_w=0;
}
}
printf("%f\n %f\n",vitesse_v,vitesse_w);

}*/
