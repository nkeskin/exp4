#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <string>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetPlan.h>
#include <std_msgs/Int8.h>
#include <thread>
#include <cstring>

using namespace std;

ros::Publisher  goal_sender;
ros::Publisher  goal_canceler;
ros::Subscriber exp4_map_sub;
ros::Subscriber status_sub;
ros::Subscriber first_moved_sub;
ros::Publisher  vel_pub;


geometry_msgs::Twist vel;
geometry_msgs::PoseStamped goal;
actionlib_msgs::GoalStatus goal_status;
actionlib_msgs::GoalID goalID;



vector<double> robotCoordI;
vector<double> robotCoordJ;
vector<geometry_msgs::PoseStamped> pos;
geometry_msgs::PoseStamped p;

//string service_name = "/move_base_node/NavfnROS/make_plan";
ros::ServiceClient serviceClient;
nav_msgs::GetPlan srv;

int     width,height; //Haritanin width, height bilgisi
double  res; // Haritanin resolution bilgisi
double  map2DOrigX,map2DOrigY; // Haritanin orijin bilgisi
double  robotX,robotY,robotYaw; // Robotun X ve Y koordinatlari
int     robotIIndex,robotJIndex; //Robotun haritada hangi indise denk geldiğini tutuan degişkenler
double  X1,X2,X3,X4; //Alan taramai icin gerekli degiskenler
double  Y1,Y2,Y3,Y4; // Alan taramai icin gerekli degiskenler
int     x1_j,x2_j,x3_j,x4_j; //Alan taramai icin gerekli degiskenler
int     y1_i,y2_i,y3_i,y4_i; //Alan taramai icin gerekli degiskenler
double  area[4] = {0}; //Taranan alanlarin boyutlarini tutan dizi
int     indexI[4] = {0}; //Robota hedef vermek icin tutulan i indislerinin toplami
int     indexJ[4] = {0}; //Robota hedef vermek icin tutulan j indislerinin toplami
double  goalX,goalY; //Robotun hedef bilgisi
double  ym,xm; //Alan isaretlemesi icin gerekli robotun baslangic noktasini tutacak degiskenler
int     firstMove = 0;//Ilk alan taramasi yapildiktan sonra bakilacak degisken
int     firsEnter = 0;
int     tmp = 0;
uint    tmpIndex = 0;
double  vectorX, vectorY;
int     steps = 0;
double  goalYaw = 0;
double  _x,_y;
bool    firstExp4 = false;
int     totalArea = 0;
double  firstGoalX = 0,firstGoalY = 0;
double  lastGoalX = 0, lastGoalY = 0;
int     backCnt = 0;


double  distan = 1.5; // Taranacak alanin yari genişliği
int     lastAreaThreshold = 115;
string  service_name = "/move_base_node/NavfnROS/make_plan";
string  picture_place = "/home/exp4/catkin_ws/src/navigator_exp4/";
int 	compute_loop_rate = 10;
double  treshold_to_goal = 0.4;
double  treshold_to_stop = 0.15;
double  angle_to_goal = 0.1;
string  picture_name = "out.pgm";
bool   	paint_robot_coords = false;
bool	paint_file = false;
string  map_topic = "/exp4_map";
string  base_topic = "/base_link";
string  cmd_topic = "/cmd_vel";
string  status_topic = "/move_base/status";
string  cancel_topic = "/move_base/cancel";




//x eksinden gelen metre bilgisini x ekseninde indise ceviren fonksiyon
int convertToIndexJ(double xMeter){
    double j;
    j = abs((map2DOrigX - xMeter)/res);
    return abs(width - (int)j - 1);
}

//y eksinden gelen metre bilgisini y ekseninde indise ceviren fonksiyon
int convertToIndexI(double yMeter){
    double i;
    i = abs((map2DOrigY - yMeter) / res);
    return (int)i;
}

//x eksinden gelen indis bilgisini x ekseninde metreye ceviren fonksiyon
double convertToMeterOnX(int j){
    double xMeter = map2DOrigX + ((width-j) * res);
    return xMeter;
}

//y eksinden gelen indis bilgisini y ekseninde metreye ceviren fonksiyon
double convertToMeterOnY(int i){
    double yMeter =  (map2DOrigY) + (i * res);
    return yMeter;
}

//Alan taramasi icin gerekli koordinatlari bulan fonksiyon
void findCoordinates(){
    X1 = robotX - distan;
    Y1 = robotY;
    X2 = robotX + distan;
    Y2 = robotY;
    X3 = robotX;
    Y3 = robotY + distan;
    X4 = robotX;
    Y4 = robotY - distan;
    x1_j = convertToIndexJ(X1);
    y1_i = convertToIndexI(Y1);
    x2_j = convertToIndexJ(X2);
    y2_i = convertToIndexI(Y2);
    x3_j = convertToIndexJ(X3);
    y3_i = convertToIndexI(Y3);
    x4_j = convertToIndexJ(X4);
    y4_i = convertToIndexI(Y4);
}


/*
    1 - 3 (i++,j--)
    3 - 2 (i--,j--)
    2 - 4 (i--,j++)//area13 = 0,index13I = 0,index13J = 0;
    4 - 1 (i++,j++)
*/


//1 ve 3 nolu alanlar arasinda kisimlari tarayan fonksiyon
void computeArea_13(int **map,int y1_i,int x1_j){
    int indexGap = (int)(distan / res);
    int i = y1_i;
    for(int k=0;k<indexGap+1;k++){
        for(int j=robotJIndex;j<x1_j;j++){
            if((j < width && i < height) && (i > -1 && j > -1)){
                if(map[i][j] == 0){ // 0 lar ve 25 leri karistirma
                    area[0]   += 1;
                    indexI[0] += i;
                    indexJ[0] += j;
                    map[i][j] = 25;
                }
            }
        }

        y1_i++;
        i = y1_i;
        if(i >= height)
            i = 0;
    }

    //indexI[0] = (int)(indexI[0] / area[0]);
    //indexJ[0] = (int)(indexJ[0] / area[0]);

}

//3 ve 2 nolu alanlar arasin15da kisimlari tarayan fonksiyon
void computeArea_32(int **map,int y3_i,int x3_j){
    int indexGap = (int)(distan / res);
    int j = x3_j;
    for(int k=0;k<indexGap+1;k++){
        for(int i=robotIIndex;i<y3_i;i++){
            if((j < width && i < height) && (i > -1 && j > -1)){
                if(map[i][j] == 0){ // 0 lar ve 25 leri karistirma
                    area[1]   += 1;
                    indexI[1] += i;
                    indexJ[1] += j;
                    map[i][j] = 25;
                }
            }
        }

        x3_j--;
        j = x3_j;
        if(j < 0)
            j = 0;
    }

    //indexI[1] = (int)(indexI[1] / area[1]);
    //indexJ[1] = (int)(indexJ[1] / area[1]);

}

//2 ve 4 nolu alanlar arasinda kisimlari tarayan fonksiyon
void computeArea_24(int **map,int y2_i,int x2_j){
    int indexGap = (int)(distan / res);
    int i = y2_i;
    for(int k=0;k<indexGap+1;k++){
        for(int j=x2_j+1;j<robotJIndex;j++){
            if((j < width && i < height) && (i > -1 && j > -1)){
                if(map[i][j] == 0){ // 0 lar ve 25 leri karistirma
                    area[2]   += 1;
                    indexI[2] += i;
                    indexJ[2] += j;
                    map[i][j] = 25;
                }
            }
        }

        y2_i--;
        i = y2_i;
        if(i < 0)
            i = 0;
    }

    //indexI[2] = (int)(indexI[2] / area[2]);
    //indexJ[2] = (int)(indexJ[2] / area[2]);


}

//4 ve 1 nolu alanlar arasinda kisimlari tarayan fonksiyon
void computeArea_41(int **map,int y4_i,int x4_j){
    int indexGap = (int)(distan / res);
    int j = x4_j;
    for(int k=0;k<indexGap+1;k++){
        for(int i=y4_i+1;i<robotIIndex;i++){
            if((j < width && i < height) && (i > -1 && j > -1)){
                if(map[i][j] == 0){ // 0 lar ve 25 leri karistirma
                    area[3]   += 1;
                    indexI[3] += i;
                    indexJ[3] += j;
                    map[i][j]  = 25;
                }
            }
        }

        x4_j++;
        j = x4_j;
        if(j >= width)
            j = 0;
    }

    //indexI[3] = (int)(indexI[3] / area[3]);
    //indexJ[3] = (int)(indexJ[3] / area[3]);

}

//Taranan alanlar arasinda en buyugunun indisini bulan fonksiyon
int findMaxArea(){
    int max = 0;
    int inx;

    totalArea = area[0] + area[1] + area[2] + area[3];

    area[0] += area[1];
    area[1] += area[2];
    area[2] += area[3];
    area[3] += area[0];

    int tmp1 = (indexI[0] + indexI[1]);
    int tmp2 = (indexI[1] + indexI[2]);
    int tmp3 = (indexI[2] + indexI[3]);
    int tmp4 = (indexI[3] + indexI[0]);

    indexI[0] = (int)(tmp1/area[0]);
    indexI[1] = (int)(tmp2/area[1]);
    indexI[2] = (int)(tmp3/area[2]);
    indexI[3] = (int)(tmp4/area[3]);

    tmp1 = (indexJ[0] + indexJ[1]);
    tmp2 = (indexJ[1] + indexJ[2]);
    tmp3 = (indexJ[2] + indexJ[3]);
    tmp4 = (indexJ[3] + indexJ[0]);

    indexJ[0] = (int)(tmp1/area[0]);
    indexJ[1] = (int)(tmp2/area[1]);
    indexJ[2] = (int)(tmp3/area[2]);
    indexJ[3] = (int)(tmp4/area[3]);


    for(int i = 0;i < 4 ;i++){
        if(area[i] > max){
            max = area[i];
            inx = i;
        }
    }

    for(int i =0;i<4;i++){
        area[i] = 0;
    }

    return inx;
}

//Gezilen alanlarin bir daha gezilmemesini saglayan fonsksiyon
void markMap(int **map){
    int first = 0;
    int start_i,finish_i;
    int start_j,finish_j;
    if(tmp == 0){
        ym = robotY;
        xm = robotX;
        tmp = 1;
    }
    int ym_i = convertToIndexI(ym);
    int xm_j = convertToIndexJ(xm);
    int cncl = height;
    for(int i=0;i<height;i++){ // kim matriste ilk geliyor onu bul
        for(int j=0;j<width;j++){
            if((i == ym_i) && (j == xm_j)){
                first = 1;
            }else if((i == robotIIndex) && (j == robotJIndex)){//burayi kesin olmayana göre yap 4. ler yerine robotIIndex ve robotJIndex yaz
                first = 2;
            }
        }
        if(first != 0)
            i = cncl + 10;
    }


    if(first == 1){
        start_i = ym_i;
        start_j = xm_j;
        finish_i = robotIIndex;
        finish_j = robotJIndex;
    }else if(first == 2){
        start_i = robotIIndex;
        start_j = robotJIndex;
        finish_i = ym_i;
        finish_j = xm_j;
    }



    if(height > width){
        for(int i=start_i;i<finish_i;i++){
            for(int j=0;j<width;j++){
                if(map[i][j] == 0)
                    map[i][j] = 250;
            }
        }
    }else if(width >= height){
        for(int j=start_j;j<finish_j;j++){
            for(int i=0;i<height;i++){
                if(map[i][j] == 0)
                    map[i][j] = 250;
            }
        }
    }
}

//Haritayi, hedefleri ve alan taramasini resim olarak yazdiran fonksiyon
void paintFile(int **map){
    string picture = picture_place + picture_name;
    FILE *f = fopen(picture.c_str(), "wb");
    fprintf(f, "P6\n%i %i 255\n", width, height);

    for (int i=0; i<height; i++){
        for (int j=0;j<width;j++){
            int skip = 0;
            if(paint_robot_coords){
            	for(uint k = 0; k < robotCoordI.size(); k++){//Robotun gectigi yerleri isaretleyen kisim
	                if((i == convertToIndexI(robotCoordI[k])) && (j == convertToIndexJ(robotCoordJ[k]))){
	                    fputc(255, f);
	                    fputc(0, f);
	                    fputc(0, f);
	                    skip = 1;
	                    k = robotCoordI.size();
	                }
            	}
            }
            if(skip == 0){
            if((i == convertToIndexI(goalY)) && (j == convertToIndexJ(goalX))){//Hedef mor ile renklendirildi
                fputc(75, f);
                fputc(0, f);
                fputc(130, f);
            }else if((i == convertToIndexI(_y)) && (j == convertToIndexJ(_x))){//Hedef mor ile renklendirildi
                fputc(0, f);
                fputc(255, f);
                fputc(0, f);
            }else if((i == y1_i) && (j == x1_j)){//1. nokta mavi ile renklendirildi
                    fputc(0, f);
                    fputc(0, f);
                    fputc(255, f);
                }else if((i == y2_i) && (j == x2_j)){//2. nokta siyah ile renklendirildi
                    fputc(0, f);
                    fputc(0, f);
                    fputc(0, f);
                }else if((i == y3_i) && (j == x3_j)){//3. nokta gri ile renklendirildi
                    fputc(55, f);
                    fputc(55, f);
                    fputc(55, f);
                }else if((i == y4_i) && (j == x4_j)){//4. nokta yesil ile renklendirildi
                    fputc(0, f);
                    fputc(255, f);
                    fputc(0, f);
                }else if(map[i][j] == 0){ // Gezilebilir alan beyaz ile renklendirildi
                fputc(255, f);
                fputc(255, f);
                fputc(255, f);
            }else if(map[i][j] == 100){ // Duvarlar mavi ile renklendirildi
                fputc(0, f);
                fputc(0, f);
                fputc(255, f);
            }else if(map[i][j] == -1){ //Bilinmeyen alanlar sari ile renklendirildi
                fputc(255, f);
                fputc(255, f);
                fputc(0, f);
            }else if (map[i][j] == 250){ //Robotun gectigi alanlar bordo ile renklendirildi
                fputc(19, f);
                fputc(19, f);
                fputc(19, f);
            }else if (map[i][j] == 25){ //Robotun taradigi alanlar gri ile renklendirildi
                fputc(55, f);
                fputc(55, f);
                fputc(55, f);
            }
            
           }
        }
    }

    fclose(f);
}



void doComputeMarkPaint(int **mapData){

    findCoordinates();

    //Alanlar isaretleniyor
    markMap(mapData);

    //Gerekli alan hesaplamalari yapiliyor
    computeArea_13(mapData,y1_i,x1_j);
    //cout << "Area13: " << area[0] << " - index13I: "  << indexI[0] << " - index13J: " << indexJ[0] << endl;

    computeArea_32(mapData,y3_i,x3_j);
    //cout << "Area32: " << area[1] << " - index32I: "  << indexI[1] << " - index32J: " << indexJ[1] << endl;

    computeArea_24(mapData,y2_i,x2_j);
    //cout << "Area24: " << area[2] << " - index24I: "  << indexI[2] << " - index24J: " << indexJ[2] << endl;

    computeArea_41(mapData,y4_i,x4_j);
    //cout << "Area41: " << area[3] << " - index41I: "  << indexI[3] << " - index41J: " << indexJ[3] << endl;


    //Gol icin en uygun alan hesaplaniyor
    int inx = findMaxArea();

    goalX = convertToMeterOnX(indexJ[inx]);
    goalY = convertToMeterOnY(indexI[inx]);

    for(int i=0;i<4;i++){
        indexI[i] = 0;
        indexJ[i] = 0;
    }

    /*
    //Gollerin koordinatlarinin resimde cikmasi saglanıyor
    robotCoordI.push_back(goalY);
    robotCoordJ.push_back(goalX);
    */

    //Robotun gectigi yerlerin resimde cikmasi saglaniyor
    robotCoordI.push_back(goalX);
    robotCoordJ.push_back(goalY);

    //cout << "goalX: " << goalX << " - goalY: "  << goalY << endl;

    //Resim yazdiriliyor
    if(paint_file)
    	paintFile(mapData);

}

void hasAnyWallNeigbour(int **mapData,double goalx,double goaly){
    int i = convertToIndexI(goaly);
    int j = convertToIndexJ(goalx);

    int wallDist = 2;

    if((i < height-wallDist && i > wallDist-1) && (j < width-wallDist && j > wallDist-1)){
        if(mapData[i-wallDist][j-wallDist] == 100){
            goal_status.status = 4;
            goalX = convertToMeterOnX(j+wallDist);
            goalY = convertToMeterOnY(i+wallDist);
        }else if(mapData[i-wallDist][j+wallDist]){
            goal_status.status = 4;
            goalX = convertToMeterOnX(j+wallDist);
            goalY = convertToMeterOnY(i-wallDist);
        }else if(mapData[i+wallDist][j-wallDist]){
            goal_status.status = 4;
            goalX = convertToMeterOnX(j-wallDist);
            goalY = convertToMeterOnY(i+wallDist);
        }else if(mapData[i+wallDist][j+wallDist]){
            goal_status.status = 4;
            goalX = convertToMeterOnX(j-wallDist);
            goalY = convertToMeterOnY(i-wallDist);
        }else if(mapData[i][j-wallDist]){
            goal_status.status = 4;
            goalX = convertToMeterOnX(j+wallDist);
            goalY = convertToMeterOnY(i);
        }else if(mapData[i][j+wallDist]){
            goal_status.status = 4;
            goalX = convertToMeterOnX(j-wallDist);
            goalY = convertToMeterOnY(i);
        }else if(mapData[i-wallDist][j]){
            goal_status.status = 4;
            goalX = convertToMeterOnX(j);
            goalY = convertToMeterOnY(i+wallDist);
        }else if(mapData[i+wallDist][j]){
            goal_status.status = 4;
            goalX = convertToMeterOnX(j);
            goalY = convertToMeterOnY(i-wallDist);
        }
    }
}

bool callPlanningService(ros::ServiceClient &serviceClient)
{

    bool serviceOkay = false;
    srv.request.start.header.frame_id = "map";
    srv.request.start.pose.position.x = robotX;
    srv.request.start.pose.position.y = robotY;
    srv.request.start.pose.orientation.w = 1.0;

    srv.request.goal.header.frame_id = "map";
    srv.request.goal.pose.position.x = goalX;
    srv.request.goal.pose.position.y = goalY;
    srv.request.goal.pose.orientation.w = 1.0;

    srv.request.tolerance = 0.2;

    while (!ros::service::waitForService(service_name, ros::Duration(2.0))) {
        ROS_INFO("Waiting for service move_base/make_plan to become available");
    }

    // Perform the actual path planner call
    if (serviceClient.call(srv)) {
        if (!srv.response.plan.poses.empty()) {
            serviceOkay = true;
            for(uint i = 0;i < srv.response.plan.poses.size();i++) {
                robotCoordI.push_back(srv.response.plan.poses.at(i).pose.position.y);
                robotCoordJ.push_back(srv.response.plan.poses.at(i).pose.position.x);
                //cout << "poseX - poseY : "  << srv.response.plan.poses.at(i).pose.position.x << "  -  " << srv.response.plan.poses.at(i).pose.position.y << endl;

            }
        }
        else {
            ROS_WARN("Got empty plan");
        }
    }
    else {
        ROS_ERROR("Failed to call service %s - is the robot moving?", serviceClient.getService().c_str());
    }

    return serviceOkay;
}


double get_direction_yaw(double _x, double _y) {
    double cur_pos_x = robotX;
    double cur_pos_y = robotY;
    double dir_point_x = cur_pos_x + cos(robotYaw);
    double dir_point_y = cur_pos_y + sin(robotYaw);

    double vector_org_to_dir_x = dir_point_x - cur_pos_x;
    double vector_org_to_dir_y = dir_point_y - cur_pos_y;
    double vector_org_to_target_x = _x - cur_pos_x;
    double vector_org_to_target_y = _y - cur_pos_y;
    /*  exploits vector product between the vectors (v1 = from current_position to direction v2 = from current_position to target)
     to figure out whether the target lies on the left or right of the robot */
    double result = vector_org_to_dir_x * vector_org_to_target_y - vector_org_to_dir_y * vector_org_to_target_x;
    // std::cout << "Result for rotation: " << result << std::endl;
    if ( result < 0)
        return -1.0;
    else
        return 1.0;
}

double calculate_radians_forTurn(double _x, double _y) {
    double cur_pos_x = robotX;
    double cur_pos_y = robotY;
    // Target direction with respect to current position of robot = target - current_pose
    double tx_wrt_poseOrg = _x - cur_pos_x;
    double ty_wrt_poseOrg = _y - cur_pos_y;
    double dir_x = cos(robotYaw);
    double dir_y = sin(robotYaw);
    // std::cout << "dir_x: " << dir_x << ", dir_y: " << dir_y << std::endl;
    // Cosine formula to find degree between direction the robot is heading to and target direction
    double numerator = tx_wrt_poseOrg * dir_x + ty_wrt_poseOrg * dir_y;
    double denominator = sqrt(tx_wrt_poseOrg * tx_wrt_poseOrg + ty_wrt_poseOrg * ty_wrt_poseOrg) * sqrt(dir_x * dir_x + dir_y * dir_y);
    double radians_to_turn = acos(numerator / denominator);
    // std::cout << "Radian to turn: " << radians_to_turn << std::endl;
    return radians_to_turn;
}


int adjustDirectionYaw(double goalx, double goaly) {

    geometry_msgs::Twist twist_msg_tmp;
    twist_msg_tmp.linear.x = 0.0;
    //ros::Rate loop_rate(10);

    if(0.1 < calculate_radians_forTurn(goalx, goaly) && ros::ok()) {
        double rotation_tmp = get_direction_yaw(goalx, goaly);
        twist_msg_tmp.angular.z = rotation_tmp * 0.25;
        // std::cout << "Publishing angular: " << twist_msg_tmp.angular.z << std::endl;
        vel_pub.publish(twist_msg_tmp);
        //loop_rate.sleep();
        return -1;
    }else
        return 1;

    twist_msg_tmp.angular.z = 0.0;
    vel_pub.publish(twist_msg_tmp);

    //ROS_INFO("\nDone adjusting robot's direction!\n");
}

double calculateDistance(double goalx, double goaly, geometry_msgs::PoseStamped p){
    double substraction = sqrt(pow((goalx-p.pose.position.x),2) + pow((goaly-p.pose.position.y),2));
    return substraction;
}


int moveForward(geometry_msgs::PoseStamped p){
    geometry_msgs::Twist twist_msg_tmp;
    twist_msg_tmp.linear.x = 0;
    if(calculateDistance(robotX,robotY,p) > 0.1){
        twist_msg_tmp.linear.x = 0.15;
        ros::Duration(1).sleep();
        vel_pub.publish(twist_msg_tmp);
        return -1;
    }else
        return 1;
    twist_msg_tmp.linear.x = 0.0;
    vel_pub.publish(twist_msg_tmp);
}


void computeGoals(const nav_msgs::OccupancyGrid::ConstPtr& msg){

    findCoordinates();

    int sayac = 0;

    //Harita icin yer aciliyor
    int **mapData = new int*[height];
    for(int i = 0; i < height; i++){
        mapData[i] = new int[width];
    }

    //Harita bilgisi okunuyor
    for(int i = 0;i < height; i++ ){
        for(int j=width-1;j>=0;j--){
            mapData[i][j] = msg->data[sayac];
            sayac++;
        }
    }

    markMap(mapData);

    //Gerekli alan hesaplamalari yapiliyor
    computeArea_13(mapData,y1_i,x1_j);
    //cout << "Area13: " << area[0] << " - index13I: "  << indexI[0] << " - index13J: " << indexJ[0] << endl;

    computeArea_32(mapData,y3_i,x3_j);
    //cout << "Area32: " << area[1] << " - index32I: "  << indexI[1] << " - index32J: " << indexJ[1] << endl;

    computeArea_24(mapData,y2_i,x2_j);
    //cout << "Area24: " << area[2] << " - index24I: "  << indexI[2] << " - index24J: " << indexJ[2] << endl;

    computeArea_41(mapData,y4_i,x4_j);
    //cout << "Area41: " << area[3] << " - index41I: "  << indexI[3] << " - index41J: " << indexJ[3] << endl;

    //Gol icin en uygun alan hesaplaniyor
    int inx = findMaxArea();

    goalX = convertToMeterOnX(indexJ[inx]);
    goalY = convertToMeterOnY(indexI[inx]);

    hasAnyWallNeigbour(mapData,goalX,goalY);

    //paintFile(mapData);

    for(int i=0;i<4;i++){
        indexI[i] = 0;
        indexJ[i] = 0;
    }

    for(int i = 0; i < height; i++) {
        delete [] mapData[i];
    }
    delete [] mapData;

}


void computeThread(){

    ros::Time now = ros::Time(0);
    tf::TransformListener listener;
    tf::StampedTransform  transform;

    ros::Rate loop_rate(compute_loop_rate);

    while(ros::ok()){
        if(firstExp4){
            try {
                /*listener.waitForTransform("/exp4_map", "/base_link", now, ros::Duration(3.0));
                listener.lookupTransform("/exp4_map", "/base_link", now, transform);*/
                listener.waitForTransform(map_topic, base_topic, now, ros::Duration(2.0));
                listener.lookupTransform(map_topic, base_topic, now, transform);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }


            //if(firstMove == 1){

            //cout << "Im here" << endl;
            robotX = transform.getOrigin().x();//robotun x eksenindeki metrik bilgisi
            robotY = transform.getOrigin().y();//robotun y eksenindeki metrik bilgisi
            robotYaw = tf::getYaw(transform.getRotation());
            //cout << "Robot x: " << robotX << "   - Roboty : " << robotY << endl;
            robotIIndex = convertToIndexI(robotY); //Robotun i indisi
            robotJIndex = convertToIndexJ(robotX); //Robotun j indisi


            if(steps == 1){
                if(calculateDistance(robotX,robotY,srv.response.plan.poses.at(tmpIndex)) > treshold_to_goal){
                    p = srv.response.plan.poses.at(tmpIndex);
                    //cout << "poseX - poseY : "  << p.pose.position.x << "  -  " << p.pose.position.y << endl;
                    //cout << "robotYaw - goalYaw: " << robotYaw << "   -   "  << goalYaw <<  endl;
                    steps = 2;
                }else
                    tmpIndex++;
                if(tmpIndex >= srv.response.plan.poses.size()){
                    tmpIndex = 0;
                    steps = 0;
                }

            }

            if(steps == 2){
                if((calculate_radians_forTurn(p.pose.position.x,p.pose.position.y) > angle_to_goal)){
                    //double goalYaw = findGoalYaw(msg);
                    //cout << " Hedef - robot yaw : " << goalYaw - robotYaw << endl;
                    vel.angular.z = get_direction_yaw(p.pose.position.x,p.pose.position.y) * 0.5;
                    vel_pub.publish(vel);
                }else{
                    steps = 3;
                    vel.angular.z = 0;
                    vel_pub.publish(vel);
                }

            }


            if(steps == 3){
                double substraction = sqrt(pow((p.pose.position.x-robotX),2) + pow((p.pose.position.y-robotY),2));
                //cout << "Goal X - Y: " << goalX << " - " << goalY << endl;
                //cout << "Substraction: " << substraction << endl;
                if(substraction > treshold_to_stop){
                    vel.linear.x = 0.2;
                    vel_pub.publish(vel);
                }else{
                    vel.linear.x = 0;
                    vel_pub.publish(vel);
                    steps = 1;
                }
            }
        }

        loop_rate.sleep();
    }
    

}

void moveBackGoal(){
    if(totalArea < lastAreaThreshold){
        if(backCnt == 0){
            goalX = firstGoalX;
            goalY = firstGoalY;
            lastGoalX = robotX;
            lastGoalY = robotY;
            backCnt = 1;
        }
    }

    if(backCnt == 1){
        goalX = lastGoalX;
        goalY = lastGoalY;
        backCnt = 2;
    }else if(backCnt == 2){
        goalX = firstGoalX;
        goalY = firstGoalY;
    }
}

//Gerekli hesaplamalari yapip gol ureten callback
void exp4Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){

    //cout << "Total area: " << totalArea << endl;

    width = msg->info.width;
    height = msg->info.height;
    res = msg->info.resolution;
    map2DOrigX = msg->info.origin.position.x;//Matrisin 0,0 indisinin x koordinati
    map2DOrigY = msg->info.origin.position.y;//Matrisin 0,0 indisinin y koordinati

    /*cout << "header    : "  << msg->header.frame_id << endl;
    cout << "height-i  : "  << height << endl;
    cout << "width-j   : "  << width << endl;
    cout << "resulotion: "    << res << endl;
    cout << "Matrixoriginx: "    << map2DOrigX  << endl;
    cout << "Matrixoriginy: "    << map2DOrigY << endl;*/

    if(firstMove == 1){
        if(!firstExp4){
            firstExp4 = true;
            ros::Time now = ros::Time(0);
            tf::TransformListener listener;
            tf::StampedTransform  transform;
            try {
                /*listener.waitForTransform("/exp4_map", "/base_link", now, ros::Duration(3.0));
                listener.lookupTransform("/exp4_map", "/base_link", now, transform);*/
                listener.waitForTransform(map_topic, base_topic, now, ros::Duration(2.0));
                listener.lookupTransform(map_topic, base_topic, now, transform);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }

            //cout << "Im here" << endl;
            robotX = transform.getOrigin().x();//robotun x eksenindeki metrik bilgisi
            robotY = transform.getOrigin().y();//robotun y eksenindeki metrik bilgisi
            robotYaw = tf::getYaw(transform.getRotation());
            //cout << "Robot x: " << robotX << "   - Roboty : " << robotY << endl;
            robotIIndex = convertToIndexI(robotY); //Robotun i indisi
            robotJIndex = convertToIndexJ(robotX); //Robotun j indisi
            firstGoalX = robotX;
            firstGoalY = robotY;
        }

        if(steps == 0){

            computeGoals(msg);

           // moveBackGoal();

            if(callPlanningService(serviceClient))
                steps = 1;

        }
   }

}


//Gol veren callback
void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){

    goal.header.frame_id = "exp4_map";
    goal.header.stamp = ros::Time::now();

    if(firstMove == 1){

        if(msg->status_list.empty()){
            goal.pose.position.x = goalX;
            goal.pose.position.y = goalY;
            goal.pose.position.z = 0.0;
            goal.pose.orientation.x = 0.0;
            goal.pose.orientation.y = 0.0;
            goal.pose.orientation.z = 0.0;
            goal.pose.orientation.w = 1.0;

            goal_sender.publish(goal);
        }else
            goal_status = msg->status_list.back();

        if(goal_status.status == 3 || goal_status.status == 4){//3 SUCCEEDED, 4 ABORTED
            goal.pose.position.x = goalX;
            goal.pose.position.y = goalY;
            goal.pose.position.z = 0.0;
            goal.pose.orientation.x = 0.0;
            goal.pose.orientation.y = 0.0;
            goal.pose.orientation.z = 0.0;
            goal.pose.orientation.w = 1.0;
            goal_sender.publish(goal);
        }else{

            goal.pose.position.x = goalX;
            goal.pose.position.y = goalY;
            goal.pose.position.z = 0.0;
            goal.pose.orientation.x = 0.0;
            goal.pose.orientation.y = 0.0;
            goal.pose.orientation.z = 0.0;
            goal.pose.orientation.w = 1.0;
            goal_sender.publish(goal);
        }

    }
    
}

void firstMovedCallback(const std_msgs::Int8::ConstPtr& msg){
    firstMove = msg->data;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "navigator_exp4");
    ros::NodeHandle n;
    //int test = 11;
    ros::NodeHandle nh("~");

    nh.getParam("lastAreaThreshold",lastAreaThreshold);
    cout << "lastAreaThreshold : " << lastAreaThreshold << endl;
    nh.getParam("service_name",service_name);
    cout << "service_name : " << service_name << endl;
    nh.getParam("distan",distan);
    ROS_INFO("distan %.3f",distan);
    nh.getParam("picture_place",picture_place);
    cout << "picture_place : " << picture_place << endl;
    nh.getParam("map_topic",map_topic);
    cout << "map_topic : " << map_topic << endl;
    nh.getParam("base_topic",base_topic);
    cout << "base_topic : " << base_topic << endl;
    nh.getParam("cmd_topic",cmd_topic);
    cout << "cmd_topic : " << cmd_topic << endl;
    nh.getParam("status_topic",status_topic);
    cout << "status_topic : " << status_topic << endl;
    nh.getParam("cancel_topic",cancel_topic);
    cout << "cancel_topic : " << cancel_topic << endl;
	nh.getParam("compute_loop_rate",compute_loop_rate);
    cout << "compute_loop_rate : " << compute_loop_rate << endl;
	nh.getParam("treshold_to_goal",treshold_to_goal);
    ROS_INFO("treshold_to_goal: %.3f",treshold_to_goal);
    nh.getParam("treshold_to_stop",treshold_to_stop);
    ROS_INFO("treshold_to_stop: %.3f",treshold_to_stop);
    nh.getParam("angle_to_goal",angle_to_goal);
    ROS_INFO("angle_to_goal: %.3f",angle_to_goal);
    nh.getParam("picture_name",picture_name);
    cout << "picture_name : " << picture_name << endl;
    nh.getParam("paint_robot_coords",paint_robot_coords);
    cout << "paint_robot_coords : " << paint_robot_coords << endl;
    nh.getParam("paint_file",paint_file);
    cout <<"paint_file : " << paint_file << endl;

    serviceClient = n.serviceClient<nav_msgs::GetPlan>(service_name, true);

    first_moved_sub = n.subscribe("/first_moved",1,firstMovedCallback);
    exp4_map_sub    = n.subscribe("/exp4_map", 1, exp4Callback);
    goal_sender     = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    goal_canceler   = n.advertise<actionlib_msgs::GoalID>(cancel_topic,1);
    status_sub      = n.subscribe(status_topic, 1, statusCallback);
    vel_pub         = n.advertise<geometry_msgs::Twist>(cmd_topic, 10);
    std::thread goal_thread(computeThread);
    //ros::spin();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    goal_thread.join();

}
