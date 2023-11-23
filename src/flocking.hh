#ifndef FLOCKING_ROBOT_HH
#define FLOCKING_ROBOT_HH
#include "robot.hh"
#include "cmath"



class FlockingRobot;
// Definition of a pointer to an aggregation robot (unnecessary)
typedef std::shared_ptr<FlockingRobot> FlockingRobotPtr;

class FlockingRobot : public mrs::Robot {
public:

  // Constructor for the Flocking Robot
  FlockingRobot(unsigned int id, const mrs::Position2d & p, 
		const mrs::RobotSettings & settings = mrs::defaultRobotSettings,
		const mrs::Velocity2d & vel = mrs::Velocity2d::Random()):
    mrs::Robot(id, p, settings)
  {
    m_vel = vel;
  }

  // Method to compute the action
  const mrs::Velocity2d & action(const std::vector<mrs::RobotPtr> & swarm){
    
    int vecinos;
    vecinos = swarm.size();

    //std::cout << id() << " " << swarm[0]->position()<< std::endl; 
    //std::cout << id() << " " << (*swarm[0]).position()<< std::endl; 

    
    

    std::vector<mrs::Position2d> v;
    std::vector<float> ang;
    std::vector<float> angD;
    std::vector<float> W;

    //std::cout << "A1" << vecinos << std::endl;
    for(int i=0; i<vecinos; i = i+1)
    {
      v.push_back(swarm[i]->position());
      
      ang.push_back(atan2(swarm[i]->velocity()[1],swarm[i]->velocity()[0]));
      
      //std::cout << mrs::distance(swarm[i]->position(),m_pos) << std::endl;

      angD.push_back(mrs::angle(m_pos,swarm[i]->position()));
      
      W.push_back(1/pow(mrs::distance(swarm[i]->position(),m_pos),2));
    }
    
    if (vecinos!=0){
      mrs::Position2d mp;
      
      mp = mrs::average(v);
      
      float k1;
      k1 = 1;
      float ang1;
      ang1 = mrs::angle(m_pos,mp);
      
      float mang;
      mang = mrs::averageAngle(ang);
      float k2;
      k2 = 1;
      //std::cout << "A5" << std::endl;

      float mangW;
      mangW = mrs::averageAngleW(angD, W);
      mangW = M_PI+mangW;
      float k3;
      k3 = 1;

      

      
      
      float P_Ang;
      std::vector<float> P_Ang1;
      std::vector<float> AngW;
      float P_Ang2;

      P_Ang1 = {ang1,mang,mangW};
      AngW = {k1,k2,k3};

      P_Ang2 = mrs::averageAngleW(P_Ang1,AngW);
      
      P_Ang=-atan2(m_vel[1],m_vel[0])+(P_Ang2);

      P_Ang = mrs::wrap2pi(P_Ang)*0.5;
      //P_Ang = 0+atan2(m_vel[1],m_vel[0]);

      //std::cout << id() << " " << vecinos<< std::endl; 
      //std::cout << ang1 << std::endl;
      //std::cout << P_Ang << std::endl;

      Eigen::Matrix<float,2,2> R;
      R(0,0) = cos(P_Ang);
      R(0,1) = -sin(P_Ang);
      R(1,0) = sin(P_Ang);
      R(1,1) = cos(P_Ang);

      //std::cout << R << std::endl;

      m_vel = R * m_vel;
    }
    

    

    return m_vel;
  }

  //void rndVel() { m_vel = mrs::Velocity2d::Random(); }

  // Name of the type of flocking robot
  std::string name() const {return std::string("Flk");}

  // Method to clone the flocking robot
  mrs::RobotPtr clone() const{
    mrs::RobotPtr ptr = std::make_shared<FlockingRobot>(m_id,
							m_pos,
							m_settings,
              m_vel);
    return ptr;
  }
private:
  
};


#endif
