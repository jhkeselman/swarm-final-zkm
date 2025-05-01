#include "foraging_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <footbot_foraging.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

CForagingLoopFunctions::CForagingLoopFunctions() :
   m_cForagingArenaSideX(-1.7f, 1.7f),
   m_cForagingArenaSideY(-1.7f, 1.7f),
   m_pcFloor(NULL),
   m_pcRNG(NULL),
   m_unCollectedFood(0),
   m_nEnergy(0),
   m_unEnergyPerFoodItem(1),
   m_unEnergyPerWalkingRobot(1) {
}
/*
*
* OUR NEW FUNCTIONS START
*
*/

bool inRadius(const CVector2& c_position_on_plane, float buffer = 0) {
   float radius = 0.5 + buffer;
   float x = c_position_on_plane.GetX();
   float y = c_position_on_plane.GetY(); 

   return x*x + y*y < radius*radius;
}

SFoodItem CForagingLoopFunctions::generateFoodItem() {
   CVector2 samplePos;
   do {
      samplePos = CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
               m_pcRNG->Uniform(m_cForagingArenaSideY));
   } while(inRadius(samplePos, 0.25));
   
   SFoodItem sItem;
   sItem.Position = samplePos;
   sItem.Progress = 100; // 100 time steps
   sItem.Reward = m_pcRNG->Uniform(CRange<Real>(0, 100));
   sItem.Assigned = 0;

   return sItem;
}

void CForagingLoopFunctions::loadFood() {
   /* Check whether a robot is on a food item */
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

   // For each footbot
   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {
      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(cFootBot.GetControllableEntity().GetController());

      /* Get food data */
      CFootBotForaging::SFoodData& sFoodData = cController.GetFoodData();
      
      // Update global information with local data assignment
      for(SFoodItem& globalItem : m_cFoodItems) {
         for(SFoodItem& localItem : sFoodData.localData) {
             if(localItem.Position == globalItem.Position) {
                 globalItem.Assigned = localItem.Assigned || globalItem.Assigned;
             }
         }
     }

      //copy everything
      sFoodData.globalData = m_cFoodItems;

      cController.timestep = argos::CSimulator::GetInstance().GetSpace().GetSimulationClock();
   }
}

void CForagingLoopFunctions::deleteFoodItem(int idx) {
   /* Check whether a robot is on a food item */
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

   // For each footbot
   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {
      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(cFootBot.GetControllableEntity().GetController());

      /* Get food data */
      CFootBotForaging::SFoodData& sFoodData = cController.GetFoodData();
      
      // Populate each food structure with locations of all foods
      sFoodData.globalData.erase(sFoodData.globalData.begin() + idx);
   }
}

/*
*
* OUR NEW FUNCTIONS END
*
*/

/****************************************/
/****************************************/

void CForagingLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      TConfigurationNode& tForaging = GetNode(t_node, "foraging");
      /* Get a pointer to the floor entity */
      m_pcFloor = &GetSpace().GetFloorEntity();
      /* Get the number of food items we want to be scattered from XML */
      UInt32 unFoodItems;
      GetNodeAttribute(tForaging, "items", unFoodItems);
      /* Get the number of food items we want to be scattered from XML */
      GetNodeAttribute(tForaging, "radius", m_fFoodSquareRadius);
      m_fFoodSquareRadius *= m_fFoodSquareRadius;
      /* Create a new RNG */
      m_pcRNG = CRandom::CreateRNG("argos");
      /* Distribute uniformly the items in the environment */
      for(UInt32 i = 0; i < unFoodItems; ++i) {
         SFoodItem sItem = generateFoodItem();
         m_cFoodItems.push_back(sItem);
      }
      /* Get the output file name from XML */
      GetNodeAttribute(tForaging, "output", m_strOutput);
      /* Open the file, erasing its contents */
      m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
      m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
      /* Get energy gain per item collected */
      GetNodeAttribute(tForaging, "energy_per_item", m_unEnergyPerFoodItem);
      /* Get energy loss per walking robot */
      GetNodeAttribute(tForaging, "energy_per_walking_robot", m_unEnergyPerWalkingRobot);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Reset() {
   /* Zero the counters */
   m_unCollectedFood = 0;
   m_nEnergy = 0;
   /* Close the file */
   m_cOutput.close();
   /* Open the file, erasing its contents */
   m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
   m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
   /* Distribute uniformly the items in the environment */
   for(UInt32 i = 0; i < m_cFoodItems.size(); ++i) {
      CVector2 samplePos;
      do {
         samplePos = CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                  m_pcRNG->Uniform(m_cForagingArenaSideY));
      } while(inRadius(samplePos, 0.25));
      
      m_cFoodItems[i].Position.Set(samplePos.GetX(),
                        samplePos.GetY());
   }

   loadFood();
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Destroy() {
   /* Close the file */
   m_cOutput.close();
}

/****************************************/
/****************************************/

CColor CForagingLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
   if(inRadius(c_position_on_plane)) { // -1.0f
      return CColor::GRAY50;
   }
   for(UInt32 i = 0; i <  m_cFoodItems.size(); ++i) {
      if((c_position_on_plane - m_cFoodItems[i].Position).SquareLength() < m_fFoodSquareRadius) {
         return CColor::BLACK;
      }
   }
   return CColor::WHITE;
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::PreStep() {
   /* Logic to pick and drop food items */
   /*
    * If a robot is in the nest, drop the food item
    * If a robot is on a food item, pick it
    * Each robot can carry only one food item per time
    */
   UInt32 unWalkingFBs = 0;
   UInt32 unRestingFBs = 0;
   /* Check whether a robot is on a food item */
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {
      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(cFootBot.GetControllableEntity().GetController());
      /* Count how many foot-bots are in which state */
      if(! cController.IsResting()) ++unWalkingFBs;
      else ++unRestingFBs;
      /* Get the position of the foot-bot on the ground as a CVector2 */
      CVector2 cPos;
      cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      /* Get food data */
      CFootBotForaging::SFoodData& sFoodData = cController.GetFoodData();
      /* The foot-bot has a food item */
      if(sFoodData.HasFoodItem) {
         /* Check whether the foot-bot is in the nest */
         if(inRadius(cPos)) {
            /* Place a new food item on the ground */
            // CVector2 samplePos;
            // do {
            //    samplePos = CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
            //             m_pcRNG->Uniform(m_cForagingArenaSideY));
            // } while(inRadius(samplePos, 0.25));
            // m_cFoodPos[sFoodData.FoodItemIdx].Set(samplePos.GetX(),
            //                                       samplePos.GetY());
            /* Drop the food item */
            sFoodData.HasFoodItem = false;
            sFoodData.FoodItemIdx = 0;
            ++sFoodData.TotalFoodItems;
            /* Increase the energy and food count */
            m_nEnergy += m_unEnergyPerFoodItem;
            ++m_unCollectedFood;
            /* The floor texture must be updated */
            m_pcFloor->SetChanged();
         }
      }
      else {
         /* The foot-bot has no food item */
         /* Check whether the foot-bot is out of the nest */
         if(!inRadius(cPos)) {
            /* Check whether the foot-bot is on a food item */
            bool bDone = false;
            for(size_t i = 0; i < m_cFoodItems.size() && !bDone; ++i) {
               if((cPos - m_cFoodItems[i].Position).SquareLength() < m_fFoodSquareRadius) {
                  if(m_cFoodItems[i].Progress == 0) {
                     /* If so, we move that item out of sight */
                     m_cFoodItems[i].Position.Set(100.0f, 100.f);
                     totalReward += m_cFoodItems[i].Reward;
                     LOG << "Total Reward: " << totalReward << std::endl;
                     deleteFoodItem(i);
                     /* The foot-bot is now carrying an item */
                     sFoodData.HasFoodItem = true;
                     sFoodData.FoodItemIdx = i;
                     /* The floor texture must be updated */
                     m_pcFloor->SetChanged();
                     /* We are done */
                     bDone = true;
                  }
                  m_cFoodItems[i].Progress--;
               }
            }
         }
      }
   }
   /* Update energy expediture due to walking robots */
   m_nEnergy -= unWalkingFBs * m_unEnergyPerWalkingRobot;
   /* Output stuff to file */
   m_cOutput << GetSpace().GetSimulationClock() << "\t"
             << unWalkingFBs << "\t"
             << unRestingFBs << "\t"
             << m_unCollectedFood << "\t"
             << m_nEnergy << std::endl;
   
   /*
   * CHECK IF NEW FOOD NEEDS TO BE POPULATED
   */
   int tStep = argos::CSimulator::GetInstance().GetSpace().GetSimulationClock();
   // Every few time steps update
   int recurrance = 50;
   if(tStep % recurrance == 0) {
      SFoodItem sItem = generateFoodItem();
      m_cFoodItems.push_back(sItem);
      m_pcFloor->SetChanged();
   }
   loadFood();
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CForagingLoopFunctions, "foraging_loop_functions")
