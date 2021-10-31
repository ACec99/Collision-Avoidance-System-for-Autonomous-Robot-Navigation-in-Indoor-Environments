# Prog_LABIAGI_2021
Di seguito i passi per eseguire il progetto:
- eseguire il web control per far partire la mappa e rviz utilizzando le seguenti istruzioni:
  1)cd ~/labiagi_2020_21/workspaces/srrg2_labiagi/src/srrg2_navigation_2d/config
  2)source ~/labiagi_2020_21/workspaces/srrg2_labiagi/devel/setup.bash
  3)~/labiagi_2020_21/srrg2_webctl/proc_webctl run_navigation.webctl
  4) andare alla pagina web con indirizzo http://localhost:9001/
  5) schiacciare i primi cinque bottoni nella tabella presente nella pagina aperta al punto precedente
- aprire un altro terminale, entrare nella cartella Prog_LABIAGI_2021 facendo cd Prog_LABIAGI_2021 ed eseguire catkin_make
- fare source devel/setup.bash
- eseguire il comando rosrun collision_avoidance CollisionMain
- aprire un altro terminale per inviare i comandi di velocità al robot ( bisogna lanciare: rostopic pub /cmd_vel_AUX /geometry_msgs/Twist -r 1 <message>)