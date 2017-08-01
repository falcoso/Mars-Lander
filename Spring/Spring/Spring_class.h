#ifndef SPRING
#define SPRING
#include <vector>
class spring {
public:
  std::vector<double> v_list, x_list, t_list;
  double m, k, x, v;

  void dynamics(double t_max, double dt);
  spring();
  ~spring();

};
#endif
