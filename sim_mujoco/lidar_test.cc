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
  int index = m->body_dofadr[slinam_id];
  if (slinam_id >= 0) {
    if (keyinputs.up) {
      d->qvel[index + 1] = speed; 
    }
    else if (keyinputs.down) {
      d->qvel[index + 1] = -speed; 
    }
    else {
      d->qvel[index + 1] = 0.0; 
    }
    if (keyinputs.right) {
      d->qvel[index + 0] = speed; 
    }
    else if (keyinputs.left) {
      d->qvel[index + 0] = -speed; 
    }
    else {
      d->qvel[index + 0] = 0.0; 
    }
  }
}

#define PORT 2000
#define IP_ADDR "127.0.0.1"
#define MAX_SAMP 5
#define DEF_PROTOCOL_NUM 10
#define NUM_TEST_DATA 2
#define DEBUG_SOCKET 0
#define DEBUG_RAYCAST_HIT_POINTS 0
#define DEBUG_LIMIT_TEST 0

static void send_socket(
  int socket_desc,
  struct sockaddr_in *server_addr,
  uint8_t *data_to_send,
  size_t data_size
) {

#if DEBUG_SOCKET
  // printf(" | %02X", byte_to_send);
  for (int i = 0; i < data_size; i++) {
    printf(" | %02x", data_to_send[i]);
  }
#endif

  if (!sendto(socket_desc, data_to_send, data_size, 0,
         (struct sockaddr*)server_addr, sizeof(*server_addr))) {
    std::printf("ERROR: unable to send packet\n");
  }
}

static void send_socket_udp(double start_ang, double end_ang, std::vector<uint16_t> samp_dists) {

  if (MAX_SAMP <= 1) {
    printf("number of distance samples is too small\n");
    exit(EXIT_FAILURE);
  }

  int socket_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(PORT);
  server_addr.sin_addr.s_addr = inet_addr(IP_ADDR);

#if DEBUG_SOCKET
  printf("start packet :\n");
#endif

  uint8_t ph_lsb = 0xAA;
  uint8_t ph_msb = 0x55;

  uint8_t ct = 0x00;
  uint8_t lsn = samp_dists.size();

  uint8_t fsa_lsb = (uint8_t)start_ang;
  uint8_t fsa_msb = (uint16_t)start_ang >> 8;

  uint8_t lsa_lsb = (uint8_t)end_ang;
  uint8_t lsa_msb = (uint16_t)end_ang >> 8;

  size_t packet_size = DEF_PROTOCOL_NUM + lsn*2;
  uint8_t *data = (uint8_t *)malloc(packet_size);
  if (!data) {
    printf("FAILED TO ALLOCATE MEMORY for data to send");
    exit(EXIT_FAILURE);
  }
  data[0] = ph_lsb;
  data[1] = ph_msb;
  data[2] = ct;
  data[3] = lsn;
  data[4] = fsa_lsb;
  data[5] = fsa_msb;
  data[6] = lsa_lsb;
  data[7] = lsa_msb;

  // checksum calc as xor of all protocols
  uint8_t cs_lsb = ph_lsb ^ ct ^ fsa_lsb ^ lsa_lsb;
  uint8_t cs_msb = ph_msb ^ lsn ^ fsa_msb ^ lsa_msb;
  for (uint16_t v : samp_dists) {
    uint8_t dist_lsb = (uint8_t)(v & 0xFF);
    uint8_t dist_msb = (uint8_t)((v >> 8) & 0xFF);
    cs_lsb ^= dist_lsb;
    cs_msb ^= dist_msb;
  }
  data[8] = cs_lsb;
  data[9] = cs_msb;

  for (int i = 0; i < samp_dists.size(); i++) {
    uint16_t v = samp_dists[i];
    uint8_t dist_lsb = (uint8_t)(v & 0xFF);
    uint8_t dist_msb = (uint8_t)((v >> 8) & 0xFF);
    data[i*2 + DEF_PROTOCOL_NUM] = dist_lsb; 
    data[i*2 + DEF_PROTOCOL_NUM + 1] = dist_msb; 
  }
  send_socket(socket_desc, &server_addr, data, packet_size);
  close(socket_desc);
  free(data);

#if DEBUG_SOCKET
  printf("\n : cs lsb: %02X, msb: %02X end packet\n", cs_lsb, cs_msb);
#endif
}

#if DEBUG_RAYCAST_HIT_POINTS
static void move_test_point(double x, double y) {
  int test_id = mj_name2id(m, mjOBJ_BODY, "test_point1");
  if (test_id == -1) {
    printf("FOUND NOTHING");
    return;
  }
  int jntid = m->body_jntadr[test_id];
  if (jntid == -1) {
    printf("FOUND NOTHING2");
    return;
  }
  int index = m->jnt_qposadr[jntid];
  if (index == -1) {
    printf("FOUND NOTHING3");
    return;
  }
  printf("index for new point: %d\n", index);
  d->qpos[index + 0] = x; 
  d->qpos[index + 1] = y; 
  d->qpos[index + 2] = 2;
}
#endif

static void raycast_from_slinam(double delta_time) {
  static double rad_dir = 0.0;
  static int nbytes = 0;
  static double start_ang = 0.0;
  static double end_ang = 0.0;
  // static uint16_t samp_dists[NUM_SAMP];
  static std::vector<uint16_t> samp_dists;


#if DEBUG_LIMIT_TEST
  static int ndata_sent = 0;
  if (ndata_sent >= NUM_TEST_DATA) {
    return;
  }
#endif

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

  double ang_deg = (rad_dir * 180 / PI);
  int adjusted_ang = (int)round(ang_deg * 64) << 1;
  adjusted_ang |= 0x1;
  if (nbytes == 0) {
    start_ang = adjusted_ang;
  }

  if (dist != -1) {
    uint16_t new_dist = (uint16_t)lround(dist * 400.0);
    samp_dists.push_back(new_dist);
#if DEBUG_RAYCAST_HIT_POINTS
    double hit_point_x = start[0] + dir[0]*dist;
    double hit_point_y = start[1] + dir[1]*dist;
    printf("ang: %f, dist: %d, hit point: %f, %f\n", rad_dir, new_dist / 4, hit_point_x, hit_point_y);
    move_test_point(hit_point_x, hit_point_y);
#endif
  }

  nbytes++;
  if (nbytes >= MAX_SAMP) {
    if (samp_dists.size() != 0) {
      end_ang = adjusted_ang;
      send_socket_udp(start_ang, end_ang, samp_dists);
#if DEBUG_LIMIT_TEST
      ndata_sent++;
#endif
      // if (ndata_sent >= NUM_TEST_DATA) {
      //   exit(EXIT_SUCCESS);
      // } 
    }
    nbytes = 0;
    start_ang = 0.0;
    end_ang = 0.0;
    samp_dists.clear();
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
