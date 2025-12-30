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
#define NUM_SAMP 5

void send_socket(int socket_desc, struct sockaddr_in *server_addr, uint32_t byte_to_send) {
  printf(" | %02X", byte_to_send);
  // if (!sendto(socket_desc, &byte_to_send, sizeof(byte_to_send), 0,
  //        (struct sockaddr*)server_addr, sizeof(*server_addr))) {
  //   std::printf("ERROR: unable to send packet\n");
  // }
}

static void send_socket_udp(double start_ang, double end_ang, uint16_t samp_dists[NUM_SAMP]) {
  int socket_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(PORT);
  server_addr.sin_addr.s_addr = inet_addr(IP_ADDR);

  // int res = bind(socket_desc, (struct sockaddr*)&server_addr, sizeof(server_addr));

  // if (res == -1) {
  //   std::printf("ERROR: unable to bind server\n");
  //   exit(EXIT_FAILURE);
  // }

  printf("start packet :\n");

  uint8_t ph_lsb = 0x55;
  uint8_t ph_msb = 0xAA;
  send_socket(socket_desc, &server_addr, ph_lsb);
  send_socket(socket_desc, &server_addr, ph_msb);

  uint8_t ct = 0x00;
  send_socket(socket_desc, &server_addr, ct);
  uint8_t lsn = 50;
  send_socket(socket_desc, &server_addr, lsn);

  uint8_t fsa_lsb = (uint8_t)start_ang;
  uint8_t fsa_msb = (uint16_t)start_ang >> 8;
  send_socket(socket_desc, &server_addr, fsa_lsb);
  send_socket(socket_desc, &server_addr, fsa_msb);

  uint8_t lsa_lsb = (uint8_t)end_ang;
  uint8_t lsa_msb = (uint16_t)end_ang >> 8;
  send_socket(socket_desc, &server_addr, lsa_lsb);
  send_socket(socket_desc, &server_addr, lsa_msb);

  uint8_t cs_lsb = 0;
  uint8_t cs_msb = 0;
  send_socket(socket_desc, &server_addr, cs_lsb);
  send_socket(socket_desc, &server_addr, cs_msb);

  if (NUM_SAMP <= 1) {
    printf("number of distance samples is too small\n");
    exit(EXIT_FAILURE);
  }
  for (int i = 0; i < NUM_SAMP; i++) {
    uint16_t v = samp_dists[i];

    uint8_t dist_lsb = (uint8_t)(v & 0xFF);
    uint8_t dist_msb = (uint8_t)((v >> 8) & 0xFF);
    // printf("dist lsb: %02X, msb: %02X ACTUAL VAL: %u\n",
    //    (unsigned)dist_lsb,
    //    (unsigned)dist_msb,
    //    (unsigned)samp_dists[i]);
    send_socket(socket_desc, &server_addr, dist_lsb);
    send_socket(socket_desc, &server_addr, dist_msb);
    samp_dists[i] = 0;
  }
  close(socket_desc);
  printf("\n : end packet\n");
}

static void raycast_from_slinam(double delta_time) {
  static double rad_dir = 0.0;
  static int nbytes = 0;
  static double start_ang = 0.0;
  static double end_ang = 0.0;
  static uint16_t samp_dists[NUM_SAMP];

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

  if (nbytes == 0) {
    start_ang = rad_dir;
  }
  else if (nbytes == NUM_SAMP - 1) {
    end_ang = rad_dir;
  }

  if (dist != -1) {
    // const mjtNum hit_pos[3] = {
    //   start[0] + dir[0]*dist, 
    //   start[1] + dir[1]*dist, 
    //   start[2] + dir[2]*dist, 
    // };
    // std::printf("intersecting hit_pos: %f, %f, %f\n", hit_pos[0], hit_pos[1], hit_pos[2]);

    samp_dists[nbytes] = (uint16_t)lround(dist * 100.0);
    // printf("samp dist: %d, actual dist: %f\n", samp_dists[nbytes], dist);
  }
  else {
    samp_dists[nbytes] = 0;
  }

  nbytes++;
  if (nbytes >= NUM_SAMP) {
    send_socket_udp(start_ang, end_ang, samp_dists);
    nbytes = 0;
    start_ang = 0.0;
    end_ang = 0.0;
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
