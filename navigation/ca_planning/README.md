# ca_planning packages

## Jump Point Search

https://zerowidth.com/2013/a-visual-explanation-of-jump-point-search.html

https://github.com/KumarRobotics/jps3d

https://github.com/daggarwa/AStar_Plugin_ROS/blob/master/src/astar_planner/src/AStarPlanner.cpp#L75

https://github.com/ros-planning/navigation/blob/melodic-devel/navfn/src/navfn_ros.cpp

http://wiki.ros.org/costmap_2d

https://github.com/KumarRobotics/jps3d/compare/master...ICRA2017:reproducible

https://github.com/ros-planning/navigation/issues/147

https://answers.ros.org/question/276935/move_base-make_plan-service-crashes-transport-error/

```bash
GUI=false VERBOSE=true GLOBAL_PLANNER=jps RVIZ_CONFIG=navigation LOCALIZATION=amcl roslaunch ca_gazebo create_house.launch
```

### TODO

- Understand how navfn.{h|cpp} work

setNavArr()
  ns = nx*ny;
  unsigned char* costarr = new unsigned char[ns];
  memset(costarr, 0, ns*sizeof(unsigned char));
  potarr = new float[ns];
setCostmap()
  : cmap --> input costmap
  : cm --> output costmap
  : COST_UNKNOWN_ROS 255 // unknown
  : COST_OBS 254		     // forbidden
  : COST_OBS_ROS 253	   // obstacles
  : navfn cost values are set to
  : COST_NEUTRAL + COST_FACTOR * costmap_cost_value.
  : Incoming costmap cost values are in the range 0 to 252.
  : With COST_NEUTRAL of 50, the COST_FACTOR needs to be about 0.8 to
  : ensure the input values are spread evenly over the output range, 50
  : to 253.  If COST_FACTOR is higher, cost values will have a plateau
  : around obstacles and the planner will then treat (for example) the
  : whole width of a narrow hallway as equally undesirable and thus
  : will not plan paths down the center.
  : COST_NEUTRAL 50		// open space
  : COST_FACTOR 0.8
  for (int i=0; i<ny; i++)
  {
    int k=i*nx;
    for (int j=0; j<nx; j++, k++, cmap++, cm++)
    {
      *cm = COST_OBS; // All start as forbidden
      int v = *cmap;
      if (v < COST_OBS_ROS) --> Difuse
      {
        v = COST_NEUTRAL+COST_FACTOR*v; // 50 + 80% * v
        std::min(v, COST_OBS_ROS)
        *cm = v;
      }
      else if(v == COST_UNKNOWN_ROS) // Unknown --> obstacles
      {
        v = COST_OBS_ROS;
        *cm = v;
      }
    }
setStart()
  int start[2]
setGoal()
  int goal[2]
calcNavFnDijkstra()
  setupNavFn()
    : Set up "potarr" for new propagation
    // reset values in propagation arrays
    for (int i=0; i<ns; i++) potarr[i] = 1.0e10;
    // outer bounds of cost array
    // https://github.com/ros-planning/navigation/blob/melodic-devel/navfn/src/navfn.cpp#L375-L388
    // set goal
    int k = goal[0] + goal[1]*nx;
    initCost(k,0);
      potarr[k] = 0;
  propNavFnDijkstra(std::max(nx*ny/20, nx+ny));
    : main propagation function
  len = calcPath(nx*ny/2)
    : path construction
    : Some sanity checks:
    : 1. Stuck at same index position
    : 2. Doesn't get near goal
    : 3. Surrounded by high potentials
  return len > 0
