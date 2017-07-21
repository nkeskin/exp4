#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>
#include <thread>

using namespace std;

ros::Publisher  vel_pub;
ros::Publisher  first_moved_pub;
ros::Subscriber exp4_sub;

std_msgs::Int8          first_moved;
geometry_msgs::Twist    vel;


int     width,height;
double  res,map2DOrigX,map2DOrigY;
double  robotX,robotY,robotYaw,goalX,goalY;
int     robotIIndex,robotJIndex;
double  distan = 1.5; // Taranacak alanin yari genişliği, cok yuksek olmasi durumunda seg_fault verebilmektedir
double  X1,X2,X3,X4; //Alan taramai icin gerekli degiskenler
double  Y1,Y2,Y3,Y4; // Alan taramai icin gerekli degiskenler
int     x1_j,x2_j,x3_j,x4_j; //Alan taramai icin gerekli degiskenler
int     y1_i,y2_i,y3_i,y4_i; //Alan taramai icin gerekli degiskenler
double  area[4] = {0}; //Taranan alanlarin boyutlarini tutan dizi
int     indexI[4] = {0}; //Robota hedef vermek icin tutulan i indislerinin toplami
int     indexJ[4] = {0}; //Robota hedef vermek icin tutulan j indislerinin toplami
int     steps = 0;
double  goalYaw = 0;
double  tmpYaw = 0;

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

//3 ve 2 nolu alanlar arasinda kisimlari tarayan fonksiyon
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



//Haritayi, hedefleri ve alan taramasini resim olarak yazdiran fonksiyon
void paintFile(int **map){
    FILE *f = fopen("/home/exp4/catkin_ws/src/navigator_exp4/first.pgm", "wb");
    fprintf(f, "P6\n%i %i 255\n", width, height);

    for (int i=0; i<height; i++){
        for (int j=0;j<width;j++){
            if((i == convertToIndexI(goalY)) && (j == convertToIndexJ(goalX))){//Hedef mor ile renklendirildi
                fputc(255, f);
                fputc(0, f);
                fputc(0, f);
            }else if((i == y1_i) && (j == x1_j)){//1. nokta mavi ile renklendirildi
                fputc(0, f);
                fputc(255, f);
                fputc(0, f);
            }else if((i == y2_i) && (j == x2_j)){//2. nokta siyah ile renklendirildi
                fputc(0, f);
                fputc(255, f);
                fputc(0, f);
            }else if((i == y3_i) && (j == x3_j)){//3. nokta gri ile renklendirildi
                fputc(0, f);
                fputc(255, f);
                fputc(0, f);
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

    fclose(f);
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

double findGoalYaw(const nav_msgs::OccupancyGrid::ConstPtr& msg){
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

    //paintFile(mapData);

    for(int i = 0; i < height; i++) {
        delete [] mapData[i];
    }
    delete [] mapData;

    for(int i=0;i<4;i++){
        indexI[i] = 0;
        indexJ[i] = 0;
    }

    double vectorX = goalX - robotX;
    double vectorY = goalY - robotY;

    return atan(tan(vectorY/vectorX));
}

void transformThread(){



}

void firstMoveThread(){

    tf::TransformListener listener;
    tf::StampedTransform  transform;
    ros::Rate loop_rate(10);

    ros::Time now = ros::Time(0);

    /*try {
        listener.waitForTransform("/exp4_map", "/robot0/base_link", now, ros::Duration(2.0));
        listener.lookupTransform("/exp4_map", "/robot0/base_link", now, transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }



    robotX = transform.getOrigin().x();//robotun x eksenindeki metrik bilgisi
    robotY = transform.getOrigin().y();//robotun y eksenindeki metrik bilgisi
    robotYaw = tf::getYaw(transform.getRotation());
    cout << "X-Y-Yaw : " << "  -  " << robotX << "   -   " << robotY << "  -  " << robotYaw << endl;

    robotIIndex = convertToIndexI(robotY);
    robotJIndex = convertToIndexJ(robotX);*/

    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;

    while(ros::ok()){
        if(steps == 0){
            if(tmpYaw < 15){
                vel.angular.z = 0.5;
                vel_pub.publish(vel);
                tmpYaw+=0.5;
                ros::Duration(1).sleep();
                vel.angular.z = 0;
                vel_pub.publish(vel);
                cout << tmpYaw << endl;
            }else{
                tmpYaw = 0;
                steps++;
                vel.angular.z = 0;
                vel_pub.publish(vel);
            }
        }


        if(steps == 2){
            if((calculate_radians_forTurn(goalX,goalY) > 0.10)){
                //double goalYaw = findGoalYaw(msg);
                cout << " Hedef - robot yaw : " << goalYaw - robotYaw << endl;
                vel.angular.z = get_direction_yaw(goalX,goalY) * 0.5;
                vel_pub.publish(vel);
                ros::Duration(1).sleep();
                vel.angular.x = 0;
                vel_pub.publish(vel);
            }else{
                steps++;
                vel.angular.z = 0;
                vel_pub.publish(vel);
            }

        }


        if(steps == 3){
            double substraction = sqrt(pow((goalX-robotX),2) + pow((goalY-robotY),2));
            cout << "Goal X - Y: " << goalX << " - " << goalY << endl;
            cout << "Substraction: " << substraction << endl;
            if(substraction > 0.10){
                vel.linear.x = 0.25;
                vel_pub.publish(vel);
                ros::Duration(1).sleep();
                vel.angular.x = 0;
                vel_pub.publish(vel);
            }else{
                vel.linear.x = 0;
                vel_pub.publish(vel);
                steps++;
            }

        }

        if(steps == 4){
            if(tmpYaw < 15){
                vel.angular.z = 0.50;
                vel_pub.publish(vel);
                tmpYaw+=0.5;
                ros::Duration(1).sleep();
                vel.angular.z = 0;
                vel_pub.publish(vel);
            }else{
                vel.angular.z = 0;
                vel_pub.publish(vel);
                tmpYaw = 0;
                steps++;
            }
        }

        if(steps > 4 && steps < 20){
            first_moved.data = 1;
            first_moved_pub.publish(first_moved);
            steps++;
        }else if(steps > 20){
            exit(0);
        }
    }


    loop_rate.sleep();
}

void exp4Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){


    width = msg->info.width;
    height = msg->info.height;
    res = msg->info.resolution;
    map2DOrigX = msg->info.origin.position.x;//Matrisin 0,0 indisinin x koordinati
    map2DOrigY = msg->info.origin.position.y;//Matrisin 0,0 indisinin y koordinati

    /*cout << "header    : "  << msg->header.frame_id << endl;
    cout << "height-i  : "  << height << endl;
    cout << "width-j   : "  << width << endl;
    cout << "resulotion: "    << res << endl ;
    cout << "Matrixoriginx: "    << map2DOrigX  << endl;
    cout << "Matrixoriginy: "    << map2DOrigY << endl;*/

    tf::TransformListener listener;
    tf::StampedTransform  transform;
    ros::Rate loop_rate(3);

    ros::Time now = ros::Time(0);

    try {
        listener.waitForTransform("/exp4_map", "/robot0/base_link", now, ros::Duration(2.0));
        listener.lookupTransform("/exp4_map", "/robot0/base_link", now, transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }



    robotX = transform.getOrigin().x();//robotun x eksenindeki metrik bilgisi
    robotY = transform.getOrigin().y();//robotun y eksenindeki metrik bilgisi
    robotYaw = tf::getYaw(transform.getRotation());
    cout << "X-Y-Yaw : " << "  -  " << robotX << "   -   " << robotY << "  -  " << robotYaw << endl;

    robotIIndex = convertToIndexI(robotY);
    robotJIndex = convertToIndexJ(robotX);

    if(steps == 1){
        goalYaw = findGoalYaw(msg);
        steps++;
    }


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "first_move");
    ros::NodeHandle n;

    vel_pub = n.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 10);
    first_moved_pub = n.advertise<std_msgs::Int8>("/first_moved",10);
    exp4_sub = n.subscribe("/exp4_map", 10, exp4Callback);
    std::thread first_thread(firstMoveThread);
    std::thread second_thread(transformThread);
    ros::spin();

    first_thread.join();
    second_thread.join();
}
