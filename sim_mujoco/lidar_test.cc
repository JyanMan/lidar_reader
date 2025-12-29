#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

typedef struct KeyInputs {
  bool up, down, left, right;
} KeyInputs;
KeyInputs keyinputs;

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act==GLFW_PRESS) {
    switch (key) {
      case GLFW_KEY_BACKSPACE:
        mj_resetData(m, d);
        break;
      case GLFW_KEY_K:
        mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05, &scn, &cam);
        break;
      case GLFW_KEY_J:
        mjv_moveCamera(m, mjMOUSE_ZOOM, 0, 0.05, &scn, &cam);
        break;
      case GLFW_KEY_W:
        std::printf("pressed w\n");
        keyinputs.up = true;            
        break;
      case GLFW_KEY_S:
        keyinputs.down = true;            
        break;
      case GLFW_KEY_D:
        keyinputs.right = true;
        break;
      case GLFW_KEY_A:
        keyinputs.left = true;
        break;
    }
  }
  else if (act==GLFW_RELEASE) {
    switch (key) {
      case GLFW_KEY_W:
        std::printf("released w\n");
        keyinputs.up = false;
        break;
      case GLFW_KEY_S:
        keyinputs.down = false;
        break;
      case GLFW_KEY_D:
        keyinputs.right = false;
        break;
      case GLFW_KEY_A:
        keyinputs.left= false;
        break;
    }
  }
}

void slinam_movement(KeyInputs keyinputs) {
  const double speed = 0.2;
  int slinam_id = mj_name2id(m, mjOBJ_BODY, "slinam");
  if (slinam_id) {
    if (keyinputs.up) {
      d->qvel[3*slinam_id + 0] = speed; 
    }
    else if (keyinputs.down) {
      d->qvel[3*slinam_id + 0] = -speed; 
    }
    else {
      d->qvel[3*slinam_id + 0] = 0.0; 
    }
    if (keyinputs.right) {
      d->qvel[3*slinam_id + 1] = speed; 
    }
    else if (keyinputs.left) {
      d->qvel[3*slinam_id + 1] = -speed; 
    }
    else {
      d->qvel[3*slinam_id + 1] = 0.0; 
    }
  }
}


// main function
int main(int argc, const char** argv) {
  // check command-line arguments
  if (argc!=1) {
    std::printf(" USAGE:  basic modelfile\n");
    return EXIT_FAILURE;
  }

  const char* slinam_robot_dir = "./models/slinam_robot.xml";

  // load and compile model
  char error[1000] = "Could not load binary model";
  // if (std::strlen(slinam_robot_dir)>4 && !std::strcmp(slinam_robot_dir+std::strlen(slinam_robot_dir)-4, ".mjb")) {
  //   m = mj_loadModel(slinam_robot_dir, 0);
  // } else {
  //   m = mj_loadXML(slinam_robot_dir, 0, error, 1000);
  // }
  // if (!m) {
  //   mju_error("Load model error: %s", error);
  // }

  mjSpec* spec = NULL;
  mjsBody* slinam = NULL;

  spec = mj_parseXML(slinam_robot_dir, 0, error, 1000);
  if (!spec) {
    mju_error("parse xml to spec error: %s", error);
  }
  m = mj_compile(spec, NULL);
  if (!m) {
    mju_error("compile spec to model error: %s", error);
  }

  // make data
  d = mj_makeData(m);

  slinam = mjs_findBody(spec, "slinam");
  if (!slinam) {
    mju_error("slinam element is null");
  }

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  // glfwSetCursorPosCallback(window, mouse_move);
  // glfwSetMouseButtonCallback(window, mouse_button);
  // glfwSetScrollCallback(window, scroll);

  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window)) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = d->time;
    while (d->time - simstart < 1.0/60.0) {
      slinam_movement(keyinputs);
      mj_step(m, d);
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  //free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return EXIT_SUCCESS;
}
