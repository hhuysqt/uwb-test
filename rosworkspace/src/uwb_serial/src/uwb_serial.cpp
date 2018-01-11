
// serial specific
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

// ROS specific
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

/*
 * serial attribute init:
 *  8 bits, no stop bit, no flow control
 */
static int set_serial_attr(int fd, speed_t speed)
{
	struct termios tty;

	if(tcgetattr(fd, &tty) < 0){
		printf("Error: %s\n", strerror(errno));
		return -1;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
	tty.c_cflag |= (CLOCAL | CREAD | CS8);

	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if(tcsetattr(fd, TCSANOW, &tty) != 0){
		printf("Error: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

/*
 * open serial
 *  if STDIN_FILENO is closed, the serial will replace it
 */
static int open_serial(const char *portname)
{
	int fd;
  printf("Opening serial %s\n", portname);
	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if(fd < 0) {
		printf("Error: %s\n", strerror(errno));
		return fd;
	}
	set_serial_attr(fd, B115200);
	return fd;
}

/*
 * Init serial and relocate stdin to serial
 */
int serial_init(std::string serial)
{
	close(STDIN_FILENO);
	return open_serial(serial.c_str());
}

/*
 * ROS specific
 **/
enum basic_color {
	RED,
	GREEN,
	BLUE,
	YELLOW,
	CYAN,
	MAGENTA,
	WHITE
};
ros::Publisher marker_pub;
static void set_color(auto &color, int whatcolor)
{
  color.a = 1.0;
  switch (whatcolor) {
  case RED:
    color.r = 1.0f;
    color.g = 0.4f;
    color.b = 0.4f;
    break;
  case GREEN:
    color.r = 0.4f;
    color.g = 1.0f;
    color.b = 0.4f;
    break;
  case BLUE:
    color.r = 0.4f;
    color.g = 0.4f;
    color.b = 1.0f;
    break;
  case YELLOW:
    color.r = 1.0f;
    color.g = 1.0f;
    color.b = 0.4f;
    break;
  case CYAN:
    color.r = 0.4f;
    color.g = 1.0f;
    color.b = 1.0f;
    break;
  case MAGENTA:
    color.r = 1.0f;
    color.g = 0.4f;
    color.b = 1.0f;
    break;
  case WHITE:
  default:
    color.r = 1.0f;
    color.g = 1.0f;
    color.b = 1.0f;
    break;
  }
}
/*
 * Marker wrappers
 */
static void my_wait_and_publish(auto marker)
{
  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1) {
    if (!ros::ok()) {
      return;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  marker_pub.publish(marker);
}
static void my_plot(double x, double y, double z, enum basic_color color, uint32_t id)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "uwb_serial";
  marker.id = id;
  marker.lifetime = ros::Duration();
  marker.action = visualization_msgs::Marker::ADD;

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  set_color(marker.color, color);

  // Publish the marker
  my_wait_and_publish(marker);
}
static void my_text(std::string str, double x, double y, double z, double size, enum basic_color color, uint32_t id)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "uwb_serial";
  marker.id = id;
  marker.lifetime = ros::Duration();
  marker.action = visualization_msgs::Marker::ADD;

  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.text = str;
  marker.scale.z = size;

  set_color(marker.color, color);
  my_wait_and_publish(marker);
}
static void my_arrow(double sx, double sy, double sz, 
    double ex, double ey, double ez, 
    enum basic_color color, uint32_t id)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "uwb_serial";
  marker.id = id;
  marker.lifetime = ros::Duration();
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.type = visualization_msgs::Marker::ARROW;
  set_color(marker.color, color);
  marker.scale.y = 0.15;
  marker.scale.x = 0.05;
  geometry_msgs::Point ptmp;
  ptmp.x = sx, ptmp.y = sy, ptmp.z = sz;
  marker.points.push_back(ptmp);
  ptmp.x = ex, ptmp.y = ey, ptmp.z = ez;
  marker.points.push_back(ptmp);
  my_wait_and_publish(marker);
}

/*
 * ROS main function
 */
int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(100);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  std::string serial_port;
  if (ros::param::has("/uwb/serial"))
    ros::param::get("/uwb/serial", serial_port);
  else
    serial_port = std::string("/dev/ttyUSB0");

  if(serial_init(serial_port) < 0)
  	return -1;

  int nr_anchor;
  if (ros::param::has("/uwb/nr_anchor"))
    ros::param::get("/uwb/nr_anchor", nr_anchor);
  else
    nr_anchor = 3;

  while (ros::ok())
  {
    my_arrow(0,0,0, 3,0,0, GREEN, 2);
    my_arrow(0,0,0, 0,3,0, GREEN, 3);
    my_arrow(0,0,0, 0,0,3, GREEN, 4);
    my_text(std::string("X"), 3,0,0.5, 0.5, CYAN, 10);
    my_text(std::string("Y"), 0,3,0.5, 0.5, CYAN, 11);
    my_text(std::string("Z"), 0,0,3.5, 0.5, CYAN, 7);
    // Get coordinate from stdin(serial)
    char lastch, thisch = 0;
    do {
    	lastch = thisch;
    	thisch = getchar();
    } while (!(lastch == 's' && thisch == ':'));

    double x[nr_anchor+1], y[nr_anchor+1], z[nr_anchor+1];
    printf("xyz:");
    int xmm,ymm,zmm;
    int i;
    for (i = 0; i < nr_anchor; i++) {
      scanf("%d,%d,%d|", &xmm,&ymm,&zmm);
      x[i] = 0.001*xmm, y[i] = 0.001*ymm, z[i] = 0.001*zmm;
      printf("%2.3f,%2.3f,%2.3f|", x[i],y[i],z[i]);
      my_plot(x[i], y[i], z[i], RED, 64+i);
      char buff[100];
      std::snprintf(buff, 100, "%d:(%.3f,%.3f,%.3f)", i,x[i],y[i],z[i]);
      my_text(std::string(buff), x[i], y[i], z[i]+0.5, 0.2, RED, 128+i);
    }
    // and finally the tag
    scanf("%d,%d,%d|", &xmm,&ymm,&zmm);
    x[i] = 0.001*xmm, y[i] = 0.001*ymm, z[i] = 0.001*zmm;
    printf("%2.3f,%2.3f,%2.3f\r\n", x[i],y[i],z[i]);
    my_plot(x[i], y[i], z[i], CYAN, 64+i);
    char buff[100];
    std::snprintf(buff, 100, "T(%.3f,%.3f,%.3f)", x[i],y[i],z[i]);
    my_text(std::string(buff), x[i], y[i], z[i]+0.5, 0.2, CYAN, 128+i);

    r.sleep();
  }
}

