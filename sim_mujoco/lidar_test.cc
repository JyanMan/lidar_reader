#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#define PI 3.14159

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
static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
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

static void slinam_movement(KeyInputs keyinputs) {
  static const double speed = 0.2;
  int slinam_id = mj_name2id(m, mjOBJ_BODY, "slinam");
  int index = slinam_id * 3;
  if (slinam_id) {
    if (keyinputs.up) {
      d->qvel[index + 0] = speed; 
    }
    else if (keyinputs.down) {
      d->qvel[index + 0] = -speed; 
    }
    else {
      d->qvel[index + 0] = 0.0; 
    }
    if (keyinputs.right) {
      d->qvel[index + 1] = speed; 
    }
    else if (keyinputs.left) {
      d->qvel[index + 1] = -speed; 
    }
    else {
      d->qvel[index + 1] = 0.0; 
    }
  }
}

#define PORT 2000
#define IP_ADDR "127.0.0.1"
static void send_socket(uint32_t byte_to_send) {
  int socket_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(PORT);
  server_addr.sin_addr.s_addr = inet_addr(IP_ADDR);

  int res = bind(socket_desc, (struct sockaddr*)&server_addr, sizeof(server_addr));

  if (!res) {
    std::printf("ERROR: unable to bind server\n");
    exit(EXIT_FAILURE);
  }

  if (!sendto(socket_desc, &byte_to_send, sizeof(byte_to_send), 0,
         (struct sockaddr*)&server_addr, sizeof(server_addr))) {
    std::printf("ERROR: unable to send packet\n");
  }

  close(socket_desc);
}

static void send_socket_udp(double coord[2]) {
  static uint32_t nbytes = 0;
  const int max_bytes= 50;

  switch (nbytes) {
    case 0:
      send_socket(0x55);
      break;
    case 1:
      send_socket(0xAA);
      break;
  }

  nbytes++;
  if (nbytes >= max_bytes) {
    nbytes = 0;
  }
}

static void raycast_from_slinam(double delta_time) {

  static double rad_dir = 0;

  int slinam_id = mj_name2id(m, mjOBJ_BODY, "slinam");
  int index = slinam_id * 3;
  const mjtNum start[3] = {
    d->xpos[index + 0],
    d->xpos[index + 1],
    d->xpos[index + 2],
  };
  const mjtNum dir[3] = { cos(rad_dir), sin(rad_dir), 0.0 };
  
  int hit_id  = 0;
  mjtNum dist = mj_ray(m, d, start, dir, NULL, 0, slinam_id, &hit_id);

  if (dist != -1) {
    const mjtNum hit_pos[3] = {
      start[0] + dir[0]*dist, 
      start[1] + dir[1]*dist, 
      start[2] + dir[2]*dist, 
    };
    std::printf("intersecting hit_pos: %f, %f, %f\n", hit_pos[0], hit_pos[1], hit_pos[2]);
    double coord[2] = {hit_pos[0], hit_pos[1]};
    send_socket_udp(coord);
  }

  rad_dir += delta_time;
  if (rad_dir >= 2 * PI) {
    rad_dir = 0.0;
  }
}

static void fixed_update(KeyInputs keyinputs, double delta_time) {
  slinam_movement(keyinputs);
  raycast_from_slinam(delta_time);
  mj_step(m, d);
}


int main(int argc, const char** argv) {
  // check command-line arguments
  if (argc!=1) {
    std::printf(" USAGE:  basic modelfile\n");
    return EXIT_FAILURE;
  }

  const char* slinam_robot_dir = "./models/slinam_robot.xml";

  // load and compile model
  char error[1000] = "Could not load binary model";

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

  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window)) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = d->time;
    double delta_time = d->time - simstart;
    while (delta_time < 1.0/60.0) {
      fixed_update(keyinputs, delta_time);
      delta_time = d->time - simstart;
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
