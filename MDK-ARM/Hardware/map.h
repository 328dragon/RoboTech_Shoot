#ifndef __MAP_H
#define __MAP_H
#include "Kinematic.h" 
#include "planner.h"
namespace Map
{
    struct MapInfo_t
    {
        MapInfo_t() = default;
			MapInfo_t(Kinematic::odom_t startInfo)
        {
            this->odom = startInfo;
        }
     
        Kinematic::odom_t odom;
      

    };
    
    class Map_t
    {
       public:
        Map_t() = default;


    };



    extern Map::MapInfo_t MapLeft;
    extern Map::MapInfo_t MapMide;
    extern Map::MapInfo_t MapRight;








} // namespace Map
#endif
