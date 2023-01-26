#include <stdio.h>
#include <math.h>
#include "frame.hpp"
#include "point.hpp"

#include "rclcpp/rclcpp.hpp"
#include "voltron_msgs/msg/cone_bearings.hpp"   

using std::placeholders::_1;

using namespace motion_parallax;

class MotionParallaxSubscriber : public rclcpp::Node
{
public:
  MotionParallaxSubscriber()
  : Node("Motion Parallax Node")
  {
    subscription_ = this->create_subscription<voltron_msgs::msg::ConeBearings>(  
      "cone_bearings", 10, std::bind(&MotionParallaxSubscriber::bearing_callback, this, _1));
  }

private:
  void bearing_callback(const voltron_msgs::msg::ConeBearings & msg) const 
  {
    Frame f0(0.0, 0.0, 0.0, { (M_PI / 4.0), ((3 * M_PI) / 4) }, nullptr);
    Frame f1(0.0, 0.0, 100.0, { M_PI, 0.0 }, &f0);

    f1.correlate_to_prev();
    printf("correlated\n");
    f1.triangulate_all_objs();
    printf("triangulated\n");

    for (auto det : f1.detections()) {
        printf("=======\n");
        std::optional<Point> op = det.centroid;

        if (op) {
            printf("x: %f y: %f\n", op.value().x, op.value().y);
        } else {
            printf("this shit don exist\n");
        }
        printf("=======\n");
    }
  }
  rclcpp::Subscription<voltron_msgs::msg::ConeBearings>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionParallaxSubscriber>());
  rclcpp::shutdown();
  return 0;
}
