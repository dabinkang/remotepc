#include <array>
#include <unistd.h>
//
#include <wiringPi.h>
#include <termios.h>
//
#include "ros/ros.h" 
#include "od_postprocess/BoundingBox.h"
#include "od_postprocess/BoundingBoxes.h"
#include "od_postprocess/ObjectCount.h"
#include "od_postprocess/Alert.h"
#include "boost/date_time/posix_time/posix_time.hpp"

//
#define LED_PIN 12
//

class object {
    public:
    object(std::string _Class, int _xmin, int _ymin, int _xmax, int _ymax)
    : Class(_Class), xmin(_xmin), ymin(_ymin), xmax(_xmax), ymax(_ymax)
    {}

    private:
    std::string Class;
    int xmin;
    int ymin;
    int xmax;
    int ymax;
};

class SubscribeAndPublish {
    public:
    SubscribeAndPublish() {
        pub_ = n_.advertise<od_postprocess::Alert>("/alert_msg", 1);
        sub_ = n_.subscribe("/darknet_ros/bounding_boxes", 1, &SubscribeAndPublish::callback, this);
    }

    void callback(const od_postprocess::BoundingBoxes::ConstPtr& input) {
        count = count + 1;
        std::cout << count << "\n";
        od_postprocess::Alert output;
        boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
        
        std::string alert_msg;
        std::string class_msg;

        int obj_cnt = 0;
        for(auto i: input->bounding_boxes){obj_cnt = obj_cnt + 1;}

        int frined_confirmed = 0;
        int frined_notconfirmed = 0;
        int T = 0;
        int enemy_cnt = 0;
        int IFF_cnt = 0;
        digitalWrite(LED_PIN,LOW);
        for(auto i: input->bounding_boxes){
            
            std::string Class = i.Class;
            if (Class == "friend")  {
                for(auto j: input->bounding_boxes) {
                    std::string Class_ = j.Class;
                    if (Class_ == "IFF") {
                        if (is_confirmed(i,j)) {
                            frined_confirmed += 1;
                            T = 1;
                            break;
                        }
                    }
                }
                if (T == 0) {
                    frined_notconfirmed += 1;
                }
                T = 0;
            }
            if (Class == "enemy")  {
                enemy_cnt = enemy_cnt + 1;
                digitalWrite(LED_PIN,HIGH);
            }
            if (Class == "IFF")  IFF_cnt = IFF_cnt + 1;
        }
        std::cout << "\n" << frined_confirmed << " friend confirmed\n";
        std::cout << frined_notconfirmed << " friend not confirmed\n";
        
        std::cout << enemy_cnt << " enemy detected\n";
        std::cout << IFF_cnt << " IFF detected\n";
        std::cout << "Total " << obj_cnt << " object detected !!\n";
        sleep(1);
        std::cout << "\033[2J\033[1;1H";
        std::cout << "\n";

        output.Alert = alert_msg;
        output.Class = class_msg;
        pub_.publish(output);
    }

    int is_confirmed(od_postprocess::BoundingBox FRIEND, od_postprocess::BoundingBox IFF) {
        int a = IFF.xmax - IFF.xmin;
        int b = IFF.ymax - IFF.ymin;
        if (FRIEND.xmin > IFF.xmax || FRIEND.ymin > IFF.ymax || FRIEND.xmax < IFF.xmin || FRIEND.ymax < IFF.ymin) {
            return 0;
        } else {
            if (FRIEND.xmin > IFF.xmin) a = a - (FRIEND.xmin - IFF.xmin);
            if (FRIEND.ymin > IFF.ymin) b = a - (FRIEND.ymin - IFF.ymin);
            if (FRIEND.xmax < IFF.xmax) a = a - (IFF.xmax - FRIEND.xmax);
            if (FRIEND.ymax < IFF.ymax) b = b - (IFF.ymax - FRIEND.ymax);
            return float(a*b)/(IFF.xmax - IFF.xmin)/(IFF.ymax - IFF.ymin) > 0.9;
        }
    }

    private: //private으로 NodeHandle과 publisher, subscriber를 선언한다.
    ros::NodeHandle n_; 
    ros::Publisher pub_;
    ros::Subscriber sub_;
    int count=0;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscribe_and_publish");
  wiringPiSetupGpio();
  pinMode(LED_PIN, OUTPUT);
  SubscribeAndPublish SAPObject; //클래스 객체 선을 하게 되면 모든게 된다.
  ros::spin();
}
