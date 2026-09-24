#include "comms.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_console.h"
#include "esp_idf_version.h"
#include "esp_rom_sys.h"
#include "net.h"
#include "robot.h"
#include "sdkconfig.h"

static const char *TAG = "comms";

#define DEG(r) ((r) * 57.29577951308232f)
#define RAD(d) ((d) * 0.017453292519943295f)

/* Protocol-level error codes (in addition to robot_err_t codes, which are sent as-is). */
#define COMMS_ERR_SYNTAX 100
#define COMMS_ERR_NET    101

/* ---------- output helpers ---------- */

/*
 * Responses are assembled in a buffer and written with a single call, so log lines from other tasks
 * cannot end up in the middle of a response line.
 */
static char s_line[1024];
static size_t s_len;

static void out_begin(void)
{
    s_len = 0;
    s_line[0] = '\0';
}

__attribute__((format(printf, 1, 2))) static void out(const char *fmt, ...)
{
    /* One byte is always kept free for the final newline. */
    const size_t cap = sizeof(s_line) - 1;
    if (s_len >= cap - 1) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(s_line + s_len, cap - s_len, fmt, ap);
    va_end(ap);
    if (n > 0) {
        s_len += (size_t)n;
        if (s_len > cap - 1) {
            s_len = cap - 1;
        }
    }
}

static void out_end(void)
{
    s_line[s_len++] = '\n';
    fwrite(s_line, 1, s_len, stdout);
    fflush(stdout);
}

static int reply_robot(robot_err_t err)
{
    if (err == ROBOT_OK) {
        out("ok");
    } else {
        out("err %d %s", (int)err, robot_err_str(err));
    }
    return 0;
}

static int reply_syntax(const char *usage)
{
    out("err %d usage: %s", COMMS_ERR_SYNTAX, usage);
    return 0;
}

static void print_joints(const char *key, const float *v, int n)
{
    out(" %s=", key);
    for (int i = 0; i < n; i++) {
        out(i ? ",%.3f" : "%.3f", (double)v[i]);
    }
}

/* ---------- argument parsing ---------- */

static bool parse_float(const char *s, float *out)
{
    char *end;
    const float v = strtof(s, &end);
    if (end == s || *end != '\0') {
        return false;
    }
    *out = v;
    return true;
}

typedef struct {
    float speed_pct;
    float accel_pct;
    kin_elbow_t elbow;
} move_opts_t;

/*
 * Parses `n` positional floats followed by options -v <pct>, -a <pct>, -e up|down.
 * Returns false on any syntax error.
 */
static bool parse_move_args(int argc, char **argv, int n, float *values, move_opts_t *opts)
{
    *opts = (move_opts_t){.speed_pct = 50.0f, .accel_pct = 50.0f, .elbow = KIN_ELBOW_UP};
    if (argc < 1 + n) {
        return false;
    }
    for (int i = 0; i < n; i++) {
        if (!parse_float(argv[1 + i], &values[i])) {
            return false;
        }
    }
    for (int i = 1 + n; i < argc; i++) {
        if (i + 1 >= argc) {
            return false;
        }
        const char *opt = argv[i];
        const char *val = argv[++i];
        if (strcmp(opt, "-v") == 0) {
            if (!parse_float(val, &opts->speed_pct)) {
                return false;
            }
        } else if (strcmp(opt, "-a") == 0) {
            if (!parse_float(val, &opts->accel_pct)) {
                return false;
            }
        } else if (strcmp(opt, "-e") == 0) {
            if (strcmp(val, "up") == 0) {
                opts->elbow = KIN_ELBOW_UP;
            } else if (strcmp(val, "down") == 0) {
                opts->elbow = KIN_ELBOW_DOWN;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }
    return true;
}

/* ---------- commands ---------- */

static int cmd_status(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    robot_status_t st;
    robot_get_status(&st);
    const uint32_t mhz = esp_rom_get_cpu_ticks_per_us();
    out("ok state=%s ref=%d moving=%d estop_in=%d queue=%" PRIu32, robot_state_str(st.state), st.referenced,
           st.moving, st.estop_input, st.queued);
    print_joints("q", st.q_deg, KIN_NUM_JOINTS);
    out(" steps=");
    for (int i = 0; i < KIN_NUM_JOINTS; i++) {
        out(i ? ",%" PRId32 : "%" PRId32, st.steps[i]);
    }
    if (st.pose_valid) {
        out(" tcp=%.2f,%.2f,%.2f,%.2f,%.2f", (double)st.pose.x, (double)st.pose.y, (double)st.pose.z,
               (double)DEG(st.pose.pitch), (double)DEG(st.pose.roll));
    } else {
        out(" tcp=na");
    }
    if (st.gripper_on) {
        out(" grip=%.1f", (double)st.gripper_pct);
    } else {
        out(" grip=off");
    }
    out(" ticks=%" PRIu32, st.stepgen.ticks);
    out(" isr_us=%.2f/%.2f segments=%" PRIu32 " underruns=%" PRIu32 " rejected=%" PRIu32 " stalls=%" PRIu32 "\n",
           (double)st.stepgen.isr_cycles_avg / mhz, (double)st.stepgen.isr_cycles_max / mhz, st.stepgen.segments,
           st.stepgen.underruns, st.stepgen.rejected, st.stepgen.stalls);
    return 0;
}

static int cmd_config(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    const robot_config_t *c = robot_get_config();
    out("# geometry d1=%.1f a1=%.1f a2=%.1f a3=%.1f d5=%.1f mm\n", (double)c->d1_mm, (double)c->a1_mm,
           (double)c->a2_mm, (double)c->a3_mm, (double)c->d5_mm);
    for (int i = 0; i < KIN_NUM_JOINTS; i++) {
        const robot_joint_config_t *j = &c->joints[i];
        out("# J%d %-11s %" PRIu32 "x%" PRIu32 "x%.2f%s limits=[%.1f,%.1f] v=%.1f a=%.1f park=%.1f\n", i + 1,
               j->name, j->full_steps_per_rev, j->microsteps, (double)j->gear_ratio, j->invert ? " inv" : "",
               (double)j->min_deg, (double)j->max_deg, (double)j->v_max_deg_s, (double)j->a_max_deg_s2,
               (double)j->park_deg);
    }
    out("ok");
    return 0;
}

static int cmd_version(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    const esp_app_desc_t *app = esp_app_get_description();
    out("ok fw=%s idf=%s protocol=0", app->version, esp_get_idf_version());
    return 0;
}

static int cmd_enable(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return reply_robot(robot_enable());
}

static int cmd_disable(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return reply_robot(robot_disable());
}

static int cmd_zero(int argc, char **argv)
{
    if (argc == 1) {
        return reply_robot(robot_zero(NULL));
    }
    float q[KIN_NUM_JOINTS];
    if (argc != 1 + KIN_NUM_JOINTS) {
        return reply_syntax("zero [q1 q2 q3 q4 q5]");
    }
    for (int i = 0; i < KIN_NUM_JOINTS; i++) {
        if (!parse_float(argv[1 + i], &q[i])) {
            return reply_syntax("zero [q1 q2 q3 q4 q5]");
        }
    }
    return reply_robot(robot_zero(q));
}

static int cmd_movej(int argc, char **argv)
{
    float q[KIN_NUM_JOINTS];
    move_opts_t o;
    if (!parse_move_args(argc, argv, KIN_NUM_JOINTS, q, &o)) {
        return reply_syntax("movej q1 q2 q3 q4 q5 [-v pct] [-a pct]");
    }
    return reply_robot(robot_move_joints(q, o.speed_pct, o.accel_pct));
}

static int cmd_movep(int argc, char **argv)
{
    float v[5];
    move_opts_t o;
    if (!parse_move_args(argc, argv, 5, v, &o)) {
        return reply_syntax("movep x y z pitch roll [-e up|down] [-v pct] [-a pct]");
    }
    const kin_pose_t pose = {.x = v[0], .y = v[1], .z = v[2], .pitch = RAD(v[3]), .roll = RAD(v[4])};
    int bad = -1;
    const robot_err_t err = robot_move_pose(&pose, o.elbow, o.speed_pct, o.accel_pct, &bad);
    if (err == ROBOT_ERR_LIMIT && bad >= 0) {
        out("err %d %s (J%d)", (int)err, robot_err_str(err), bad + 1);
        return 0;
    }
    return reply_robot(err);
}

static int cmd_jog(int argc, char **argv)
{
    float v[2];
    move_opts_t o;
    if (!parse_move_args(argc, argv, 2, v, &o)) {
        return reply_syntax("jog joint(1-5) delta_deg [-v pct]");
    }
    return reply_robot(robot_jog((int)v[0] - 1, v[1], o.speed_pct));
}

static int cmd_park(int argc, char **argv)
{
    float v[1];
    move_opts_t o;
    if (!parse_move_args(argc, argv, 0, v, &o)) {
        return reply_syntax("park [-v pct]");
    }
    return reply_robot(robot_park(o.speed_pct));
}

static int cmd_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return reply_robot(robot_stop());
}

static int cmd_estop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return reply_robot(robot_estop());
}

static int cmd_reset(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return reply_robot(robot_reset());
}

static int cmd_fk(int argc, char **argv)
{
    float q[KIN_NUM_JOINTS];
    move_opts_t o;
    if (argc != 1 + KIN_NUM_JOINTS || !parse_move_args(argc, argv, KIN_NUM_JOINTS, q, &o)) {
        return reply_syntax("fk q1 q2 q3 q4 q5");
    }
    for (int i = 0; i < KIN_NUM_JOINTS; i++) {
        q[i] = RAD(q[i]);
    }
    kin_pose_t p;
    kin_forward(robot_get_model(), q, &p);
    out("ok x=%.3f y=%.3f z=%.3f pitch=%.3f roll=%.3f", (double)p.x, (double)p.y, (double)p.z,
           (double)DEG(p.pitch), (double)DEG(p.roll));
    return 0;
}

static int cmd_ik(int argc, char **argv)
{
    float v[5];
    move_opts_t o;
    if (!parse_move_args(argc, argv, 5, v, &o)) {
        return reply_syntax("ik x y z pitch roll [-e up|down]");
    }
    const kin_pose_t pose = {.x = v[0], .y = v[1], .z = v[2], .pitch = RAD(v[3]), .roll = RAD(v[4])};
    float q[KIN_NUM_JOINTS];
    int bad = -1;
    const kin_status_t ks = kin_inverse(robot_get_model(), &pose, o.elbow, q, &bad);
    if (ks != KIN_OK && ks != KIN_ERR_JOINT_LIMIT) {
        out("err %d %s", ks == KIN_ERR_SINGULAR ? ROBOT_ERR_SINGULAR : ROBOT_ERR_UNREACHABLE,
               kin_status_str(ks));
        return 0;
    }
    for (int i = 0; i < KIN_NUM_JOINTS; i++) {
        q[i] = DEG(q[i]);
    }
    out("ok");
    print_joints("q", q, KIN_NUM_JOINTS);
    out(" limits=%s", ks == KIN_OK ? "ok" : "violated");
    return 0;
}

static int cmd_grip(int argc, char **argv)
{
    const char *usage = "grip open|close|off|<0-100>";
    if (argc != 2) {
        return reply_syntax(usage);
    }
    if (strcmp(argv[1], "open") == 0) {
        return reply_robot(robot_gripper_open());
    }
    if (strcmp(argv[1], "close") == 0) {
        return reply_robot(robot_gripper_close());
    }
    if (strcmp(argv[1], "off") == 0) {
        return reply_robot(robot_gripper_off());
    }
    float pct;
    if (!parse_float(argv[1], &pct)) {
        return reply_syntax(usage);
    }
    return reply_robot(robot_gripper_set(pct));
}

static int cmd_wifi(int argc, char **argv)
{
#if !CONFIG_DUMBE_NET_ENABLE
    (void)argc;
    (void)argv;
    out("err %d WiFi disabled in this build", COMMS_ERR_NET);
    return 0;
#endif
    if (argc == 1) {
        net_status_t ns;
        net_get_status(&ns);
        out("ok configured=%d connected=%d ssid=%s ip=%s", ns.configured, ns.connected,
               ns.configured ? ns.ssid : "-", ns.connected ? ns.ip : "-");
        return 0;
    }
    if (argc != 2 && argc != 3) {
        return reply_syntax("wifi [ssid [password]]");
    }
    robot_status_t st;
    robot_get_status(&st);
    if (st.moving) {
        return reply_robot(ROBOT_ERR_BUSY); /* NVS write would stall the motion task */
    }
    const esp_err_t err = net_set_credentials(argv[1], argc == 3 ? argv[2] : "");
    if (err != ESP_OK) {
        out("err %d %s", COMMS_ERR_NET, esp_err_to_name(err));
        return 0;
    }
    out("ok");
    return 0;
}

typedef struct {
    const char *name;
    const char *help;
    esp_console_cmd_func_t func;
} command_t;

static const command_t s_commands[] = {
    {"status", "Robot state, joints, TCP, gripper, step generator stats", cmd_status},
    {"config", "Print the robot configuration", cmd_config},
    {"version", "Firmware, ESP-IDF and protocol versions", cmd_version},
    {"enable", "Enable the stepper drivers (arm holds position, not referenced)", cmd_enable},
    {"disable", "Disable the stepper drivers (the arm may fall!)", cmd_disable},
    {"zero", "zero [q1..q5]: declare current joint angles in deg (default: park pose)", cmd_zero},
    {"movej", "movej q1..q5 [-v pct] [-a pct]: synchronised joint move (deg)", cmd_movej},
    {"movep", "movep x y z pitch roll [-e up|down] [-v pct] [-a pct]: move to pose (mm, deg)", cmd_movep},
    {"jog", "jog joint(1-5) delta_deg [-v pct]: relative joint move", cmd_jog},
    {"park", "park [-v pct]: move to the park pose", cmd_park},
    {"stop", "Controlled stop, drops queued moves", cmd_stop},
    {"estop", "Immediate stop (latched until reset)", cmd_estop},
    {"reset", "Clear the e-stop latch", cmd_reset},
    {"fk", "fk q1..q5: forward kinematics (deg -> mm, deg)", cmd_fk},
    {"ik", "ik x y z pitch roll [-e up|down]: inverse kinematics", cmd_ik},
    {"grip", "grip open|close|off|<0-100>", cmd_grip},
    {"wifi", "wifi [ssid [password]]: status or set credentials", cmd_wifi},
};

/* Runs a command with its whole response emitted as a single write. */
static int run_command(void *context, int argc, char **argv)
{
    const command_t *cmd = (const command_t *)context;
    out_begin();
    const int ret = cmd->func(argc, argv);
    out_end();
    return ret;
}

esp_err_t comms_start(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "dumbe>";
    repl_config.max_cmdline_length = 256;
    const esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_console_new_repl_uart(&uart_config, &repl_config, &repl), TAG, "repl");

    ESP_RETURN_ON_ERROR(esp_console_register_help_command(), TAG, "help");
    for (size_t i = 0; i < sizeof(s_commands) / sizeof(s_commands[0]); i++) {
        const esp_console_cmd_t cmd = {
            .command = s_commands[i].name,
            .help = s_commands[i].help,
            .func_w_context = run_command,
            .context = (void *)&s_commands[i],
        };
        ESP_RETURN_ON_ERROR(esp_console_cmd_register(&cmd), TAG, "register %s", s_commands[i].name);
    }
    return esp_console_start_repl(repl);
}
