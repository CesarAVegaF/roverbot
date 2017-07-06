#include <bits/stdc++.h>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "std_msgs/String.h"
#define PI 3.14159265
using namespace std;
unsigned int num_points = 100;

float xBot = 30;
float yBot = 0;
float zBot, normaBot;
float norma, prodIn, cosA;
float xCloud, yCloud, zCloud;



class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;

  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "/scan", 10),
    laser_notifier_(laser_sub_,listener_, "base_link", 10)
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("cloud",1);
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {

    if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "/base_link",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(10.0))){
        return;
    }

    sensor_msgs::PointCloud cloud;
    try
    {
        projector_.transformLaserScanToPointCloud(
          "base_link",*scan_in, cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }

    printPoints(cloud);

    //scan_pub_.publish(cloud);
  }

  /**
  * Funcion para el tratamiento de la informacion de la nube de puntos
  */
  void printPoints(sensor_msgs::PointCloud cloud)
  {

    num_points = cloud.points.size();  // número de puntos en la lectura del scan

    vector<float> normaBot;
    zBot = cloud.points[0].z; // altura del robot o del laser
    // Caracteristicas del vector referencia al frente del robot
    normaBot.push_back(
	sqrt(
		pow(xBot, 2.0) +
		pow(yBot, 2.0) + 
		pow(zBot, 2.0)
	)
    ); 
    normaBot.push_back(xBot);
    normaBot.push_back(yBot);
    normaBot.push_back(zBot);

    //inicio del vector pasado posicion 0 de la nube de puntos
    vector<float> normaCloudPas; 
    normaCloudPas.push_back(
	sqrt(
		pow(cloud.points[0].x, 2.0)+
		pow(cloud.points[0].y, 2.0)+
		pow(cloud.points[0].z, 2.0)
	)
    );
    normaCloudPas.push_back(cloud.points[0].x);
    normaCloudPas.push_back(cloud.points[0].y);
    normaCloudPas.push_back(cloud.points[0].z);

    int obstaculos = 1; //número de obstaculos detectados
 
    //grupo de datos de un objeto
    vector< vector< float > > dataGroup;
    for(unsigned int i = 1; i < num_points; i++){
	// Extrae las coordenadas x, y, z de los puntos de la nube
        xCloud = cloud.points[i].x;
        yCloud = cloud.points[i].y;
        zCloud = cloud.points[i].z;

	//Vector de magnitud actual
	vector<float> normaCloud;
        normaCloud.push_back(sqrt((xCloud*xCloud)+(yCloud*yCloud)+(zCloud*zCloud)));
	normaCloud.push_back(xCloud);
	normaCloud.push_back(yCloud);
	normaCloud.push_back(zCloud);
	
	float rango = (normaCloud[0] * 5.0)/100.0; // Tolerancia de error

	// si esta dentro de la tolerancia hace parte del grupo de datos del objeto
	// Si no, hace parte del vertice del objeto, por lo tanto entra a la condicion y verifica el punto minimo
	if (normaCloudPas[0] > normaCloud[0]+rango || normaCloudPas[0] < normaCloud[0]-rango){
	    dataGroup.push_back(normaCloudPas); // añade los datos al grupo
            vector<float> minDistance = normaBot; //Distancia de comparación
	    double theta = 0; // ángulo incial
	    vector<float> floatTemp;
	    // verifica todos los datos en el grupo encontrando el menor
	    for(unsigned int j = 0; j < dataGroup.size(); j++){
		floatTemp = dataGroup[j];
		if(minDistance[0] >= floatTemp[0]){
		    minDistance = floatTemp;
		}
	    }
	    
	    // cos(theta) = InnerPorduct(u, v) / |u|*|v| 
	    norma = normaBot[0]*minDistance[0]; // |u|*|v| 

	    prodIn = (normaBot[1]*minDistance[1])+(normaBot[2]*minDistance[2])+(normaBot[3]*minDistance[3]); // InnerPorduct(u, v)

            cosA = prodIn/norma; // InnerPorduct(u, v) / |u|*|v| 

	    theta = acos(cosA) * (180 / PI); // theta = cos-1(InnerPorduct(u, v) / |u|*|v| )
	    // Verifica el angulo de donde proviene, si y+ el angulo es positivo, en caso contrario y- el angulo es negativo
	    if(yCloud < 0){
                theta *= -1;
            }	
	    // filtra casos donde esta en el borde del radar
	    if(minDistance[0] < 29.8){
	        printf("Hay %d obstaculo(s) a: %4.2f m, con un ángulo de: %4.2f. \n", obstaculos++, minDistance[0], theta);
	    }
	    dataGroup.clear(); // limpia el vector usado
	}
	normaCloudPas = normaCloud; // establece el punto actual como pasado.
    }
	
  }

};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "my_scan_to_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  
  ros::spin();
  
  return 0;
}
