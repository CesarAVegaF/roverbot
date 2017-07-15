ROVERBOT descripcion del proyecto

folder gazebo:
	Contiene archivos de configuracion en gazebo y rviz.
	
	launch:
		contiene archivo de lanzamiento del programa
	rviz:
		configuracion del programa rviz
	worlds:
		contiene los modelos de mapas que se usan en la simulacion de gazebo	

folder description:
	contiene archivos de creacion del robot y plugins para ser usados por gazebo

	mesh:
		contiene las texturas de los objetos para el robot
	urdf:
		contiene la descripcion y plugins del robot
	
folder control:
	Contiene archivos de configuracion y manipulacion del robot.

	config:
		contiene archivos de configuracion de mando (joys)

	include:
		contiene librerias usadas			

	launch:
		contiene archivo launch de control 

	src:
		contiene archivos de clases para el mando y comunicacion del topico en ros


folder comunication:
	Contiene archivos de comunicacion talker, listener y mensajes los cuales usan los topicos usados en la configuracion del robot.

	src:
		contiene archivos de clase para la comunicacion con el topico del laser

	std_msgs:
		contiene archivos de mensaje usados para la comunicacion 

====
comando de ejecucion
====

roslaunch roverbot roverbot_world.launch


====
comando de ejecucion talker
====

rosrun roverbot talkerLaser


====
comando de ejecucion listener
-- Este comando puede ejecutarse para corroborar la informacion publicada por el talker
====

rosrun roverbot listenerLaser
