#include "linelibrary/agvline.h"

/************************************************************************************
	agvline : Create class agv line and read all parameter
*************************************************************************************
Input :
--------------------------------------------------
nodeHandle  : ros node handle
publish_rate  : rate for while loop
Output :
----------------------------------------------------
no output
************************************************************************************/

agvline::agvline(ros::NodeHandlePtr nodeHandle, const int publish_rate)
{
	this->threadRate = publish_rate;
	this->agvGetparam(nodeHandle, this->paramAgvforward, "/conceptForward");
	this->agvGetparam(nodeHandle, this->paramAgvbackward, "/conceptBackward");
	this->acceLineForward.beginTime = clock();
	this->acceLineBackward.beginTime = clock();
	this->acceLineForward.presentSetting = 0;
	this->acceLineBackward.presentSetting = 0;
	
	// ros::NodeHandle n;
	// test_pub = n.advertise<std_msgs::String>("test_pub", 1);
}

/************************************************************************************
	~agvline :  Not used
*************************************************************************************
Input :
--------------------------------------------------
no input
Output :
----------------------------------------------------
no output
************************************************************************************/
agvline::~agvline() {}

/************************************************************************************
EnventEnable :  
*************************************************************************************
Input :
--------------------------------------------------
direct : phuong huong
Output :
----------------------------------------------------
no output
************************************************************************************/
void agvline::EnventEnable(int8_t direct)
{
	this->theadEnable = true;
	if(this->EventThread.joinable() == false)
		if(direct == FOR_WARD)
			this->EventThread = boost::thread(this->EventforwarddoStuff,this);
		else if(direct == BACK_WARD)
			this->EventThread = boost::thread(this->EventbackwarddoStuff,this);
	else ROS_ERROR("Event thread busy !!"); 
}

/************************************************************************************
EnventDisEnable :  
*************************************************************************************
Input :
--------------------------------------------------
no input
Output :
----------------------------------------------------
no output
************************************************************************************/
void agvline::EnventDisEnable()
{
	this->theadEnable = false;
	if(this->EventThread.joinable() == true)
		this->EventThread.join();
}

/************************************************************************************
	agvGetparam :  load parameter concept of agv 
*************************************************************************************
Input :
--------------------------------------------------
nodeHandle : ros node handle
param 			  : address of struct agvLineParam
Nodename    : the name of concept agv in  yaml file 
Output :
----------------------------------------------------
no output
************************************************************************************/
void agvline::agvGetparam(ros::NodeHandlePtr nodeHandle, agvLineParam &param,std::string Nodename)
{
	if(
		!nodeHandle->getParam("/L", param.agv.L) ||
		!nodeHandle->getParam("/R", param.agv.R) ||
		!nodeHandle->getParam("/K", param.agv.K) ||
		!nodeHandle->getParam(Nodename + "/V", param.V) ||
		!nodeHandle->getParam(Nodename + "/Lm", param.Lm) ||
		!nodeHandle->getParam(Nodename + "/PresentRunning", param.PresentRunning) ||
		!nodeHandle->getParam(Nodename + "/PresentSlow", param.PresentSlow) ||
		!nodeHandle->getParam(Nodename + "/K", param.K) ||
		!nodeHandle->getParam("/error_max", param.error_max)
	)   ROS_ERROR_STREAM("Get param from yaml file faided !!");
	else
	{
		param.PresentSlow /= 100;
		param.PresentRunning /= 100;
		ROS_INFO_STREAM("agvLine.cpp-" <<Nodename<<"/L = "<<param.agv.L<<" (Khoang cach 2 banh - m)");
		ROS_INFO_STREAM("agvLine.cpp-" <<Nodename<<"/R = "<<param.agv.R<<" (Ban kinh banh xe - m)");
		ROS_INFO_STREAM("agvLine.cpp-" <<Nodename<<"/K = "<<param.agv.K<<" (ti so truyen dong co)");
		ROS_INFO_STREAM("agvLine.cpp-" <<Nodename<<"/V = "<<param.V);
		ROS_INFO_STREAM("agvLine.cpp-" <<Nodename<<"/Lm = "<<param.Lm);
		ROS_INFO_STREAM("agvLine.cpp-" <<Nodename<<"/PresentRunning = "<<param.PresentRunning);
		ROS_INFO_STREAM("agvLine.cpp-" <<Nodename<<"/PresentSlow = "<<param.PresentSlow);
		ROS_INFO_STREAM("agvLine.cpp-" <<Nodename<<"/K = "<<param.K<<" (He so K- m)");
	}
}

void agvline::agvgGetlineData(Mlse_info receiveData, agvLineinfo &msg)
{
	msg.mlse = receiveData; 
	/* 
	* #LCP
	* The following assignment applies (see "Outputof line center points", page 40):
	* 0 => No track found
	* 2 => One track found
	* 3 => Two tracks found: Left diverter
	* 6 => Two tracks found: Left diverter
	* 7 => Three tracks found or 90 °C intersection
	*/
	msg.lcp.lcp_nummber = msg.mlse.lcp & LCP_NUM_BIT;
	/* 
	* Marker 
	* Bit 0 is the introductory character bit
	* Bit 1...4 present code 1...15
	*/
	msg.lcp.marker = msg.mlse.lcp & MAKER_NUM_BIT;
	/* 
	* True is  Sufficiently strong track detected
	* Fasle is  No track or track too weak
	*/
	msg.status.line_good = msg.mlse.status & LINE_IS_GOOD_BIT;
	/*
	* Display of magnetic field strength in accor‐dance
	*/
	msg.status.track_level = msg.mlse.status & TRACK_LEVEL_BIT;
	/*
	* Indicates whether or not the measuring rangehas been inverted
	* False => Negative positions on cable outlet side
	* True => Positive positions on cable outlet side
	*/
	msg.status.sensor_fipped = msg.mlse.status & SENSOR_FLIPPED_BIT;
	/*
	* Indicates whether the upper surface of themagnetic tape is magnetized to the north orsouth pole
	* False => North pole
	* True => South pole
	*/
	msg.status.polarity = msg.mlse.status & POLARITY_BIT;
	/* 
	* False => No code present to read
	* True => Sensor is reading code
	*/
	msg.status.reading_code = msg.mlse.status & READING_CODE_BIT;
	/* Error register */
	msg.mlse.error_register = receiveData.error_register;
	
}

void agvline::getDataAgvforward(Mlse_info receiveData)
{
	this->agvgGetlineData(receiveData, this->agvForwardInfo);
}

void agvline::getDataAgvbackward(Mlse_info receiveData)
{
	this->agvgGetlineData(receiveData, this->agvBackwardInfo);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
cmd_vel agvline::agvlinepurePursuite(agvLineParam agvparam, agvLineinfo agvInfo_msl, agvLineinfo agvInfo_mslfollow, acceparam acce)
{
	cmd_vel result;
		ROS_INFO("THUAT TOAN BAM LINE SO 1");
		///////////////////////////////////////////////////////////////////////////////
		///////////////////////////// CÁC HỆ SỐ ////////////////////////////////////////
		float k_atan = 1; // hệ số tỉ lệ đưa giá trị e về khoảng -1 đến 1 để lấy atan
		float k_scale_unit = 1; // hệ số thể hiện mức độ tỉ lệ của thang đo e, hệ số có đơn vị là m/rad
		float e = k_scale_unit*atan(k_atan*agvInfo_msl.mlse.position[1]);// (m) làm mềm và lọc tín hiệu đầu vào bằng hàm atan
		float e_follow = k_scale_unit*atan(agvInfo_mslfollow.mlse.position[1]); 
		//////////////////////////////////////////////////////////////////////////////////////////
		float k_v = 0.65; // hệ số vận tốc dài - thể hiện mức độ tăng hoặc giảm vận tốc dài so với tốc độ dài mong muốn agvparam.V(m/s)
		float e_max = 0.25;
		float delta_e_max = 0.38; //(m) do lech lon nhan giua 2 line (dung khi di tren duong thang)
		float kee = 2.46; //1.8 2.46
		float ke=1.15; // do nhay sai lech
		float kk_makeup_w=1; ///0.71 khi di chuyen tren duong tron
		///////////////////////////////////////////////////////////////////
		float delta_e =kee*abs(abs(e) - abs(e_follow));
		float k_reduce_v_e = (e_max-ke*abs(e))/e_max;
		float k_reduce_v_ee=((delta_e_max-delta_e)/delta_e_max);
		float k_reduce_v=1-(1-k_reduce_v_e)+(1-k_reduce_v_ee);
		float k_makeup_w=kk_makeup_w*(1+(1-k_reduce_v_e)+(1-k_reduce_v_ee));
		float L = agvparam.Lm;
		///////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//result.linear.x = acce.presentSetting*atan(k_v*agvparam.V- k_v_reduce*(abs(agvparam.V)/agvparam.V)*(abs(e/L))); //(m/s) - công thức tính vận tốc dài của robot         ///
		//result.linear.x = acce.presentSetting*atan(k_v*agvparam.V*((e_max-abs(e))/e_max));
		result.linear.x = acce.presentReality*atan(k_v*agvparam.V*k_reduce_v); //  acce.presentReality///acce.presentSetting (agvparam.V)  *k_reduce_v
		result.angular.z = atan(result.linear.x*(2*e/(e*e + L*L)))*k_makeup_w; // *k_makeup_w
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		ROS_INFO("/sai so dau xe:%f(m)/sai so duoi xe:%f(m)/van toc goc:%f(rad/s)/van toc dai:%f(m/s)",e,e_follow,result.angular.z,result.linear.x);
		ROS_INFO("do lech dau xe va duoi xe(m):%f",abs(abs(e) - abs(e_follow)));
		ROS_INFO("k_v_reduce_e:%f/k_v_reduce:%f",k_reduce_v_e,k_reduce_v_ee);
    	// std_msgs::String msg;
    	// msg.data = "test";
		// test_pub.publish(msg);
		// ROS_INFO("Publish the topic /chatter");
	return result;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////THUẬT TOÁN BÁM ĐƯỜNG SỐ 2/////////////////////////////////////////////////////////////////////////////////////////////////////////
cmd_vel agvline::agvlinefollowAngle(agvLineParam agvparam, agvLineinfo agvInfo_msl, agvLineinfo agvInfo_mslfollow, acceparam acce)
{
	// ROS_INFO("THUAT TOAN BAM DUONG SO 2");
	// cmd_vel result;
	// ///////////////////////////// CÁC HỆ SỐ////////////////////////////////////////
	// float k_atan = 1; // hệ số tỉ lệ đưa giá trị e về khoảng -1 đến 1 để lấy atan
	// float k_scale_unit = 1; // hệ số thể hiện mức độ tỉ lệ của thang đo e, hệ số có đơn vị là m/rad
	// float k_v = 1.2; // hệ số vận tốc dài - thể hiện mức độ tăng hoặc giảm vận tốc dài so với tốc độ dài mong muốn agvparam.V(m/s)
	// float k_v_reduce = 5; // hệ số suy giảm vận tốc dài - thể hiện mức độ giảm vận tốc dài của robot khi sai số e tăng, với e = 0 hệ thì vận tốc dài của robot bằng vận tốc dài mong muốn
	// float k_w = 2; // hệ số thể hiện mức độ thể hiện độ nhạy cảm của vận tốc góc của xe (giúp xe mau lấy lại góc về đường thiết kế)
	// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// float e = k_scale_unit*agvInfo_msl.mlse.position[1];	// Cảm biến line từ đầu theo chiều chạy chưa làm mềm bằng hàm atan
	// float e_follow = agvInfo_mslfollow.mlse.position[1];  	// Cảm biến line từ đuôi theo chiều chạy chưa làm mềm bằng hàm atan
	// float L = agvparam.Lm; // (m) khoảng cách từ cảm biến đến trục sau của banh xe
	// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// result.linear.x = acce.presentSetting*atan(k_v*agvparam.V- k_v_reduce*(abs(agvparam.V)/agvparam.V)*atan(abs(e/L)));// công thức tính vận tốc dài       ///
	// result.angular.z = (abs(agvparam.V)/agvparam.V)*k_w*atan(e/L)*acce.presentSetting;// công thức tính vận tốc góc                                  ///
	// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ROS_INFO("/sai so:%f(m)/sai so bam theo:%f/van toc goc:%f(rad/s)/ van toc dai:%f(m/s)/",e,e_follow,result.angular.z,result.linear.x);            
	// return result;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

cmd_vel agvline::agvLineOnDinhGocForward(void)
{
// 	this->bockAcceleration(this->acceLineBackward);
// 	this->acceleration(this->acceLineForward, 1);
// 	return this->agvlinefollowAngle(this->paramAgvforward, this->agvForwardInfo, this->agvBackwardInfo, this->acceLineForward);
}

cmd_vel agvline::agvLineOnDinhGocBackward(void)
{
	// this->bockAcceleration(this->acceLineForward);
	// this->acceleration(this->acceLineBackward, 1);
	// return this->agvlinefollowAngle(this->paramAgvbackward, this->agvBackwardInfo, this->agvForwardInfo, this->acceLineBackward);
}

cmd_vel agvline::agvLineforward(void)
{
	this->bockAcceleration(this->acceLineBackward);
	this->acceleration(this->acceLineForward, 1);
	return this->agvlinepurePursuite(this->paramAgvforward, this->agvForwardInfo, this->agvBackwardInfo, this->acceLineForward);
}

cmd_vel agvline::agvLinebackward(void)
{
	this->bockAcceleration(this->acceLineForward);
	this->acceleration(this->acceLineBackward, 1);
	return this->agvlinepurePursuite(this->paramAgvbackward, this->agvBackwardInfo, this->agvForwardInfo, this->acceLineBackward);
}

void agvline::agvStop()
{
	this->acceLineForward.presentSetting = 0;
	this->acceLineBackward.presentSetting = 0;
}

void agvline::acceleration(acceparam &acce, const float_t time)
{
	if (float_t(clock() - acce.beginTime) / CLOCKS_PER_SEC * 150 >= time)
	{
		if (acce.acceleration == UP_SPEED)
		{
			if (acce.presentReality < acce.presentSetting)
			{
				upSpeed(acce.presentReality, 0.0025);// he so van toc 0.005
			}
			else if (acce.presentReality > acce.presentSetting)
			{
				acce.presentReality = acce.presentSetting;
			}
		}
		else
		{
			if (acce.presentReality > acce.presentSetting)
			{
				reduceSpeed(acce.presentReality, 0.0030); // he so van toc
			}
			else if (acce.presentReality < acce.presentSetting)
			{
				acce.presentReality = acce.presentSetting;
			}
		}
		ROS_INFO("acceleration = %x acceset = %f accereality = %f",acce.acceleration,acce.presentSetting,acce.presentReality);
	}
}

void agvline::bockAcceleration(acceparam &acce)
{
	acce.presentSetting = 0;
	acce.presentReality = 0;
}

void agvline::upSpeed(double &present_speed, const double step)
{
	present_speed += step;
}

void agvline::reduceSpeed(double &present_speed, const double step)
{
	present_speed -= step;
}

void agvline::EventforwarddoStuff(void *payload)
{
	agvline *as = (agvline *)payload;
	ros::Rate loop_rate(as->threadRate);
	ROS_INFO("Event forward do Stuff is ready !!");
	while (ros::ok() && as->theadEnable == true)
	{
		if (as->agvForwardInfo.mlse.error_register == 0)
		{
			if(as->agvForwardInfo.status.line_good || as->agvForwardInfo.lcp.lcp_nummber > 0)
			{
				if (as->agvForwardInfo.mlse.position[2] > 0.00)
				{
					// This->acceLineForward.presentSetting = 0;
					if (as->areaThree)
						as->areaThree();
				}
				else if (as->agvForwardInfo.mlse.position[2] <= 0.00)
				{
					if (as->agvForwardInfo.mlse.position[0] == 0.00)
					{
						as->acceLineForward.acceleration = UP_SPEED;
						as->acceLineForward.presentSetting = as->paramAgvforward.PresentRunning;

						if (as->areaOne)
							as->areaOne();
					}
					else if (as->agvForwardInfo.mlse.position[0] < 0.00)
					{
						as->acceLineForward.acceleration = REDUCE_SPEED;
						//as->acceLineForward.presentSetting = as->paramAgvforward.PresentSlow;
						as->acceLineForward.presentSetting = 0;
						if (as->areaTwo)
							as->areaTwo();
					}
				}
				if(as->agvForwardInfo.status.track_level <3 && as->agvForwardInfo.status.track_level >0)
				ROS_WARN("Back line sensor: track too weak!!");
			} 
			else 
			{
				as->acceLineForward.acceleration = REDUCE_SPEED;
				as->acceLineForward.presentSetting = 0;
				ROS_ERROR("Front line sensor : no track!!");
			}	
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	ROS_ERROR("Thread out!!");
}

void agvline::EventbackwarddoStuff(void *payload)
{
	agvline *as = (agvline *)payload;
	ros::Rate loop_rate(as->threadRate);
	ROS_INFO("Event backward do Stuff is ready !!");
	while (ros::ok() && as->theadEnable == true)
	{
		if (as->agvBackwardInfo.mlse.error_register == 0)
		{
			if(as->agvBackwardInfo.status.line_good || as->agvBackwardInfo.lcp.lcp_nummber >0)
			{
				if (as->agvBackwardInfo.mlse.position[2] > 0.00)
				{
					//This->acceLineBackward.presentSetting = 0;
					if (as->areaThree)
						as->areaThree();
				}
				else if (as->agvBackwardInfo.mlse.position[2] <= 0.00)
				{
					if (as->agvBackwardInfo.mlse.position[0] == 0.00)
					{
						as->acceLineBackward.acceleration = UP_SPEED;
						as->acceLineBackward.presentSetting = as->paramAgvbackward.PresentRunning;
						if (as->areaOne)
							as->areaOne();
					}
					else if (as->agvBackwardInfo.mlse.position[0] < 0.00)
					{
						as->acceLineBackward.acceleration = REDUCE_SPEED;
					    //as->acceLineBackward.presentSetting = as->paramAgvbackward.PresentSlow;
						as->acceLineBackward.presentSetting = 0;
						if (as->areaTwo)
							as->areaTwo();
					}
				}
				if(as->agvBackwardInfo.status.track_level <=3 && as->agvBackwardInfo.status.track_level >0)
					ROS_WARN("Back line sensor: track too weak!!");
			}
			else 
			{
				// as->acceLineBackward.acceleration = REDUCE_SPEED;
				as->acceLineBackward.presentSetting = 0;
				ROS_ERROR("Back line sensor : no track!!");
			}
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
}