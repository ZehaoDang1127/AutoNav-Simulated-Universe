#include "nav2_smac_planner/traveled_corridor.hpp"
#include <iostream>
#include <stdexcept>
using nav2_smac_planner::TraveledCorridor;
void require(bool ok, const char * text) {if (!ok) throw std::runtime_error(text);}
int main() {
  try {
    TraveledCorridor history;
    nav2_costmap_2d::Costmap2D map(240,160,.1,-2,-2,0);
    for (int i=0;i<=100;++i) history.record(i*.1,0,"odom");
    require(!history.applyReturnMask(map,0,0,"odom",1.5,1.5,20),
      "A nearby recent goal activated return routing");
    for (int i=1;i<=50;++i) history.record(10,i*.1,"odom");
    for (int i=1;i<=100;++i) history.record(10-i*.1,5,"odom");
    unsigned int x,y; map.worldToMap(10,2,x,y); map.setCost(x,y,254);
    require(history.applyReturnMask(map,0,0,"odom",1.5,1.5,20),"Revisited goal not recognized");
    require(map.getCost(x,y)==254,"Travel history erased a freshly sensed hazard");
    map.worldToMap(5,0,x,y);require(map.getCost(x,y)==0,"Known departure corridor was closed");
    map.worldToMap(5,2.5,x,y);require(map.getCost(x,y)==254,"Untraveled shortcut remained open");
    require(!history.applyReturnMask(map,20,8,"odom",1.5,1.5,20),"Unvisited goal selected return mode");
    require(!history.applyReturnMask(map,0,0,"map",1.5,1.5,20),"Different frame reused history");
    history.record(30,30,"odom");
    require(!history.applyReturnMask(map,0,0,"odom",1.5,1.5,20),"Localization jump created a false connection");
    history.record(0,0,"map");
    require(!history.applyReturnMask(map,0,0,"odom",1.5,1.5,20),"Frame reset retained old corridor");
    TraveledCorridor global_history;
    for (int i=0;i<=100;++i) global_history.record(i*.1,0,"map");
    for (int i=1;i<=50;++i) global_history.record(10,i*.1,"map");
    for (int i=1;i<=100;++i) global_history.record(10-i*.1,5,"map");
    nav2_costmap_2d::Costmap2D projected(400,400,.1,-20,-20,0);
    TraveledCorridor::FrameTransform tf{"map","odom",-4,3,1.5707963267948966};
    require(!global_history.applyReturnMask(projected,-4,3,"odom",1.5,1.5,20),
      "Missing TF guessed a global history projection");
    projected.worldToMap(-4,8,x,y);projected.setCost(x,y,254);
    require(global_history.applyReturnMask(projected,-4,3,"odom",1.5,1.5,20,&tf),
      "Rotated and translated goal lost its visited history");
    require(projected.getCost(x,y)==254,"Frame projection erased a fresh hazard");
    projected.worldToMap(-4,10,x,y);require(projected.getCost(x,y)==0,
      "Current TF did not transform the traveled corridor");
    projected.resetMap(0,0,400,400);
    tf.x=-1;tf.y=6;
    require(global_history.applyReturnMask(projected,-1,6,"odom",1.5,1.5,20,&tf),
      "A GPS frame correction disabled a previously visited goal");
    require(global_history.frame()=="map","A query rewrote the stored history frame");
    tf.target_frame="other";
    require(!global_history.applyReturnMask(projected,-1,6,"odom",1.5,1.5,20,&tf),
      "Mismatched transform was accepted");
    std::cout<<"PASS: revisit detection, hazards, history resets and current-frame projection\n";
  } catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 1;}
}
