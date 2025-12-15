#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>

class ObstacleAvoidance {
public:
	ObstacleAvoidance() {
		// Postavljanje pocetne udaljenosti na veliku vrijednost
		targetDistance = 9999.0;
		// Postavljanje pocetnog stanja na trazenje prepreke
		currentState = State::SEARCH;

		laserSubscriber = nh.subscribe("/scan", 10, &ObstacleAvoidance::laserCallback, this);
		cmdVelPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
		odomSubscriber = n.subscribe("/x100/odom", 10, &ObstacleAvoidance::updateRobotPosition, this);
	}
	// Funkcija za obradu podataka sa senzora
	void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
	// Pronalazak najblize prepreke
	targetDistance = 9999.0;
	targetAngle = 0;
	for (size_t i = 0; i < msg->ranges.size(); ++i) {
		float range = msg->ranges[i];
		if (range < targetDistance) {
			targetDistance = range;
			targetAngle = msg->angle_min + msg->angle_increment * i;
		}
	}
	// Postavljanje zeljenog ugla
	targetAngle = targetAngle + 1.57;
	}
	// Funkcija za azuriranje pozicije robota
	void updateRobotPosition(const nav_msgs::Odometry msg) {
		robotX = msg.pose.pose.position.x;
		robotY = msg.pose.pose.position.y;
	}
	void run() {
		ros::Rate rate(10); // Frekvencija od 10 Hz
		while (ros::ok()) {
			// Provjera zahtjeva za prekidom
			if (currentState == State::TERMINATE) {
				// Izlazak iz petlje i obustava svih radnji
				break;
			}
			switch (currentState) {
				case State::SEARCH:
				search();
				break;
				case State::APPROACH:
				approach();
				break;
				case State::ROTATE:
				rotate();
				break;
				case State::LINEAR:
				linear();
				break;
			}
			ros::spinOnce();
			rate.sleep();
		}
	}
private:
	enum class State {
		SEARCH, APPROACH, ROTATE, LINEAR, TERMINATE
	};
	ros::NodeHandle nh;
	ros::NodeHandle n;
	ros::Subscriber laserSubscriber;
	ros::Publisher cmdVelPublisher;
	ros::Subscriber odomSubscriber;
	ros::Time begginTime;
	float targetDistance;
	float targetAngle;
	float robotX;
	float robotY;
	State currentState;
	float rememberedX;
	float rememberedY;
	bool coordinatesSet;
	// Funkcija koja obavlja trazanje prepreke
	void search() {
	// Ako je prepreka pronadjena, prelazak na stanje prilazak
	if (targetDistance < 9999.0) {
		currentState = State::APPROACH;
		coordinatesSet = false;
	}
	}
	// Funkcija koja obavlja prilazak prepreci
	void approach() {
		float angularTolerance = 0.2;
		float desiredAngle = 1.57;
		// Provjera poravnanja sa preprekom
		if (std::abs(targetAngle - desiredAngle) > angularTolerance) {
			// Smjer kretanja
			ROS_INFO("Poravnavam se sa preprekom");
			float direction = (targetAngle > 1.57) ? 1.0 : -1.0;
			geometry_msgs::Twist twist;
			twist.linear.x = 0.0;
			twist.angular.z = direction * 1.8;
			cmdVelPublisher.publish(twist);
		} else {
			// Linearno kretanje prema prepreci
			ROS_INFO("Prilazim prepreci");
			geometry_msgs::Twist twist;
			twist.linear.x = 0.85;
			twist.angular.z = 0.0;
			cmdVelPublisher.publish(twist);
		}
		if (targetDistance < 2) {
		// Robot se priblizio prepreci
		ROS_INFO("Prisao sam prepreci");
		currentState = State::ROTATE;
		}
	}
	// Funkcija koja obavlja rotaciju nakon prilaska
	void rotate() {
		float angularTolerance = 0.2;
		float desiredAngle = 0;
		// Provjera poravnanja sa preprekom
		if (std::abs(targetAngle - desiredAngle) > angularTolerance) {
			// Rotacija
			ROS_INFO("Rotiram se");
			float direction = (targetAngle > 0) ? 1.0 : -1.0;
			geometry_msgs::Twist twist;
			twist.linear.x = 0.0;
			twist.angular.z = direction * 1.8;
			cmdVelPublisher.publish(twist);
		} else {
			// Prepreka je sa desne strane, prebacivanje na stanje linearnog kretanja
			checkRobotPosition();
			currentState = State::LINEAR;
		}
	}
	// Funkcija koja obavlja linearno kretanje 
	void linear() {
	checkRobotPosition();
	float angularTolerance = 0.2;
	float desiredAngle = 0;
	geometry_msgs::Twist twist;
	// Provjera poravnanja sa preprekom
	if (std::abs(targetAngle - desiredAngle) < angularTolerance) {
		// Kretanje naprijed i provjera udaljenosti od prepreke
		if (targetDistance < 1.95) {
		ROS_INFO("Skreni lijevo");
		twist.linear.x = 0.5;
		twist.angular.z = 0.2;
		cmdVelPublisher.publish(twist);
		} else if (targetDistance > 2.05) {
		ROS_INFO("Skreni desno");
		twist.linear.x = 0.5;
		twist.angular.z = (-1) * 0.2;
		cmdVelPublisher.publish(twist);
		} else {
		ROS_INFO("Idem pravo");
		twist.linear.x = 0.6;
		twist.angular.z = 0.0;
		cmdVelPublisher.publish(twist);
	}
	} else {
		// Prepreka nije s desne strane, prebacivanje na stanje rotacije
		currentState = State::ROTATE;
	}
	}
	// Funkcija za pamcenje pocetne pozicije robota i provjeru trenutne pozicije
	void checkRobotPosition() {
	ros::Time currentTime = ros::Time::now();
	// Azuriranje pozicije robota
	if (!coordinatesSet) {
		rememberedX = robotX;
		rememberedY = robotY;
		begginTime = currentTime;
		coordinatesSet = true;
	} else if ((begginTime.toSec() - currentTime.toSec()) < -15) {
		// Provjera da li se robot vratio u pocetnu poziciju
		double distanceToRemembered = std::hypot(robotX - rememberedX, robotY - rememberedY);
		double distanceThreshold = 0.5; 
		if (distanceToRemembered < distanceThreshold) {
			ROS_INFO("Stigao u pocetnu poziciju");
			// Robot se vratio u pocetnu poziciju, prekid svih radnji
			currentState = State::TERMINATE;
			geometry_msgs::Twist twist;
			twist.linear.x = 0.0;
			twist.angular.z = 0.0;
			cmdVelPublisher.publish(twist);
		}
	}
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "obstacle_avoidance");
	ObstacleAvoidance obstacleAvoidance;
	obstacleAvoidance.run();
}

