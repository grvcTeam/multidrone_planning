#include <action_executer/trailer.h>

void Trailer::init(double _vehicle_yaw, Eigen::Vector3d _vehicle_pos)
{
  t.clear();
  for(uint8_t i = 0; i<filter; i++)
      t.push_back(ros::Time::now().toSec());
  
  _target_pos_.clear();
  for(uint8_t i = 0; i<filter; i++)
    _target_pos_.push_back(_vehicle_pos);

  a = _vehicle_yaw;
  R << cos(a), -sin(a),
        sin(a),  cos(a);

  vehicle_pos_ << _vehicle_pos(0), _vehicle_pos(1);
  p = vehicle_pos_- R*d*Eigen::Vector2d::UnitX();
  desired_point_ = p + R*r;
  
  p_         = p;
  a_         = a;
  rx_        = rx = rx_s+d;
  ry_        = ry; 
  altitude_  = z_s = altitude;
  azimuth_   = initial_azimuth;
  w_trailer  = 0;
  v_trailer  = 0;
  v_trailer_ << 0, 0;
  v_desired  << 0, 0;
  v_leader   << 0, 0;
  I_1_2      << 1, 0, 0,
                0, 1, 0;
}

void Trailer::run_trailer(Eigen::Vector3d _target_pos)
{
  t[0] = ros::Time::now().toSec(); // compute current time and time_step
  _target_pos_[0]= _target_pos;
  Ts.clear();
  for(uint8_t i = 0; i < filter-1; i++)
    Ts.push_back(t[0]-t[i+1]);

  _target_pos_s.clear();
  for(uint8_t i = 0; i < filter-1; i++)
    _target_pos_s.push_back(_target_pos_[0] - _target_pos_[i+1]);

  if (trailer_type == TRAILER_TYPE_INERTIAL)
    desired_point_ = originOfFormation_;
    
  else {
    if (trailer_type == TRAILER_TYPE_ORBIT || trailer_type == TRAILER_TYPE_ORBIT_GIMBAL) {
        azimuth = azimuth_ + angular_speed*Ts[0];
        rx = radius*cos(azimuth)+d;
        ry = radius*sin(azimuth);
        r  << rx, ry;
        rx_dot = -radius*angular_speed*sin(azimuth);
        ry_dot = radius*angular_speed*cos(azimuth);
        r_dot << rx_dot, ry_dot;
    } else if (trailer_type == TRAILER_TYPE_VARIABLE) {
        rx = rx_ + rx_dot*Ts[0];  
        ry = ry_ + ry_dot*Ts[0]; 

        if(rx > d-r_safety && rx < d+r_safety){  //safety area        
        area = 5*exp(-2.5/(pow(r_safety,2)-pow((rx-d),2))); 
          if (ry >= 0 && ry <= area && d !=0)            
            ry = area;
          else if(ry<0 && ry >= -area && d !=0)
            ry = -area;
        }      

        if((rx_s > rx_e && rx< rx_e+d) || (rx_s < rx_e && rx> rx_e+d)) rx_dot = 0; //finish condition
          r << rx, ry;
        r_dot << rx_dot, ry_dot;
  }

    a = a_ + w_trailer*Ts[0];    

    R << cos(a), -sin(a),
         sin(a),  cos(a);

    v_target = I_1_2*(_target_pos_s[filter-2])/Ts[filter-2];
    w_trailer = 0;
    if(d != 0)
      w_trailer = 1/d*Eigen::Vector2d::UnitY().transpose()*R.transpose()*v_target;
    v_trailer_ << Eigen::Vector2d::UnitX().transpose()*R.transpose()*v_target, 0;
    p = I_1_2*_target_pos - d * R*Eigen::Vector2d::UnitX();
    S_w_trailer <<          0,  -w_trailer,
                    w_trailer,           0;
    desired_point_ = p + R*r;

    v_desired << R*(v_trailer_ + S_w_trailer*r + r_dot);
  }

  // update
  altitude  = altitude_ + altitude_dot*Ts[0];
  
  if (z_e != 0 && !(altitude >= z_s && altitude <= z_e || altitude <= z_s && altitude >= z_e)) 
    altitude  = z_e;
  
  altitude_ = altitude;
  p_ = p;
  a_ = a;
  rx_ = rx;
  // ry_ = ry;
  azimuth_ = azimuth;

  std::rotate(t.rbegin(), t.rbegin() + 1, t.rend());
  std::rotate(_target_pos_.rbegin(), _target_pos_.rbegin() + 1, _target_pos_.rend());
}