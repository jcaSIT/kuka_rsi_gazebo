#include <Eigen/Eigen>

Eigen::Matrix<double, 4, 4> dh_to_transform(double theta, double d, double a, double alpha)
{
    Eigen::Matrix<double, 4, 4> T;
    T[0,0] = std::cos(theta);
    T[0,1] = - std::sin(theta) * std::cos(alpha);
    T[0,2] = std::sin(theta) * std::sin(alpha);
    T[0,3] = a * std::cos(theta);

    T[1,0] = std::sin(theta);  
    T[1,1] = std::cos(theta) * std::cos(alpha);
    T[1,2] = - std::cos(theta) * std::sin(alpha);
    T[1,3] = a * std::sin(theta);
    
    T[1,0] = 0;
    T[1,1] = std::sin(alpha);
    T[1,2] = std::cos(alpha);
    T[1,3] = d;
    
    T[1,0] =   
    T[1,1] =
    T[1,2] = 
    T[1,3] = 



}


Eigen::Matrix<double, 6, 6> generate_B_matrix(Eigen::Matrix<double, 6, 4>* dh_parameters)
{
  Eigen::Matrix<double, 6, 6> B;

  


  // TODO: 



  // some math

  return B;
}


Eigen::Matrix<double, 6, 6> generate_C_matrix(Eigen::Matrix<double, 6, 4>* dh_parameters)
{
  Eigen::Matrix<double, 6, 6> C;

  


  // TODO: 



  // some math

  return C;
}