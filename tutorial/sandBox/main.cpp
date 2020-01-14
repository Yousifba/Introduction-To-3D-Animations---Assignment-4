
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"

int main(int argc, char *argv[])
{
  Display *disp = new Display(1000, 800, "Wellcome");
  Renderer renderer;
  igl::opengl::glfw::Viewer viewer;

  viewer.load_meshs_4();
  viewer.build_kd_trees();
  viewer.draw_bounding_boxes();

  // ||| Assignment 3 |||
  /*int arm_length;
  while (true)
  {
	  std::cout << "Please Enter Arm Length (2 - 25): ";
	  std::cin >> arm_length;
	  if (arm_length >= 2 && arm_length <= 25)
		  break;
	  else
		  std::cout << "Error: Arm Length Must Be Between (2 - 25)!" << std::endl;
  }
  viewer.arm_length = arm_length;
  viewer.sys_init();*/


  // ||| Assignment 2 |||
  /*viewer.init_ds();
  viewer.quadric_error_handler();*/
  Init(*disp);
  renderer.init(&viewer);
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);

  
  delete disp;
}
