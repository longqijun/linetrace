#include "cmd_module.h"
#include "bt_module.h"
#include "motor_module.h"
#include "print_module.h"
#include "config_module.h"
#include "track_module.h"
#include "sensor_module.h"
#include "ram_log_module.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// 命令响应：始终输出，不受 print_module 控制
static void reply(const char* msg) {
  Serial.print(msg);
  bt_send(msg);
}

static void show_prompt() {
  reply(">> ");
}

static void handle_command(const char* cmd) {

  // --- print usb/bt/all on|off ---
  if (strcmp(cmd, "print usb on") == 0) {
    print_set_usb(true);
    reply(">>> USB print ON\r\n");
  } else if (strcmp(cmd, "print usb off") == 0) {
    print_set_usb(false);
    reply(">>> USB print OFF\r\n");
  } else if (strcmp(cmd, "print bt on") == 0) {
    print_set_bt(true);
    reply(">>> BT print ON\r\n");
  } else if (strcmp(cmd, "print bt off") == 0) {
    print_set_bt(false);
    reply(">>> BT print OFF\r\n");
  } else if (strcmp(cmd, "print file on") == 0) {
    print_set_file(true);
    reply(">>> File log ON (/track.log.0~3)\r\n");
  } else if (strcmp(cmd, "print file off") == 0) {
    print_set_file(false);
    reply(">>> File log OFF\r\n");
  } else if (strcmp(cmd, "print on") == 0) {
    print_set_usb(true);
    print_set_bt(true);
    reply(">>> USB+BT print ON\r\n");
  } else if (strcmp(cmd, "print off") == 0) {
    print_set_usb(false);
    print_set_bt(false);
    reply(">>> USB+BT print OFF\r\n");

  // --- stop ---
  } else if (strcmp(cmd, "stop") == 0) {
    track_set(false);
    motor_stop();
    reply(">>> Motor stopped\r\n");

  // --- track on/off ---
  } else if (strcmp(cmd, "track on") == 0) {
    track_set(true);
    reply(">>> Line tracking ON\r\n");
  } else if (strcmp(cmd, "track off") == 0) {
    track_set(false);
    reply(">>> Line tracking OFF\r\n");

  // --- speed N ---
  } else if (strncmp(cmd, "speed ", 6) == 0) {
    int level = atoi(cmd + 6);
    if (level < 1 || level > 40) {
      reply(">>> Speed range 1~40\r\n");
    } else {
      config_set_speed(level);
      char buf[48];
      snprintf(buf, sizeof(buf), ">>> Speed set to %d (PWM %d)\r\n",
               level, motor_level_to_pwm(level));
      reply(buf);
    }

  // --- turnspeed N (急弯外轮速度比例，百分比0~100) ---
  } else if (strncmp(cmd, "turnspeed ", 10) == 0) {
    int pct = atoi(cmd + 10);
    if (pct < 0 || pct > 100) {
      reply(">>> Turn speed range 0~100\r\n");
    } else {
      track_set_turn_ratio(pct / 100.0f);
      char buf[80];
      snprintf(buf, sizeof(buf), ">>> Turn speed set to %d%% (outer wheel ratio on sharp turn)\r\n", pct);
      reply(buf);
    }

  // --- sharpratio N (急弯内轮反转比例，百分比0~100，内部转成负值) ---
  } else if (strncmp(cmd, "sharpratio ", 11) == 0) {
    int pct = atoi(cmd + 11);
    if (pct < 0 || pct > 100) {
      reply(">>> Sharp ratio range 0~100\r\n");
    } else {
      track_set_sharp_ratio(-pct / 100.0f);
      char buf[80];
      snprintf(buf, sizeof(buf), ">>> Sharp ratio set to -%d%% (inner wheel reverse on sharp turn)\r\n", pct);
      reply(buf);
    }

  // --- mediumratio N (中转内轮速度比例，百分比0~100，介于mild和sharp之间) ---
  } else if (strncmp(cmd, "mediumratio ", 12) == 0) {
    int pct = atoi(cmd + 12);
    if (pct < 0 || pct > 100) {
      reply(">>> Medium ratio range 0~100\r\n");
    } else {
      track_set_medium_ratio(pct / 100.0f);
      char buf[80];
      snprintf(buf, sizeof(buf), ">>> Medium ratio set to %d%% (inner wheel speed on medium turn)\r\n", pct);
      reply(buf);
    }

  // --- xsharpratio N (发卡弯内轮反转比例，百分比0~100，内部转成负值，比sharp更激进) ---
  } else if (strncmp(cmd, "xsharpratio ", 12) == 0) {
    int pct = atoi(cmd + 12);
    if (pct < 0 || pct > 100) {
      reply(">>> Xsharp ratio range 0~100\r\n");
    } else {
      track_set_xsharp_ratio(-pct / 100.0f);
      char buf[80];
      snprintf(buf, sizeof(buf), ">>> Xsharp ratio set to -%d%% (inner wheel reverse on hairpin turn)\r\n", pct);
      reply(buf);
    }

  // --- 方案A：弯道降速 mediumspeed/sharpspeed/hairpinspeed N (各档前进速度系数，百分比0~100，仅bang-bang) ---
  } else if (strncmp(cmd, "mediumspeed ", 12) == 0) {
    int pct = atoi(cmd + 12);
    if (pct < 0 || pct > 100) {
      reply(">>> Medium speed range 0~100\r\n");
    } else {
      track_set_medium_speed(pct / 100.0f);
      char buf[88];
      snprintf(buf, sizeof(buf), ">>> Medium speed set to %d%% (bang-bang forward speed on medium turn CH3/CH6)\r\n", pct);
      reply(buf);
    }
  } else if (strncmp(cmd, "sharpspeed ", 11) == 0) {
    int pct = atoi(cmd + 11);
    if (pct < 0 || pct > 100) {
      reply(">>> Sharp speed range 0~100\r\n");
    } else {
      track_set_sharp_speed(pct / 100.0f);
      char buf[88];
      snprintf(buf, sizeof(buf), ">>> Sharp speed set to %d%% (bang-bang forward speed on sharp turn CH2/CH7)\r\n", pct);
      reply(buf);
    }
  } else if (strncmp(cmd, "hairpinspeed ", 13) == 0) {
    int pct = atoi(cmd + 13);
    if (pct < 0 || pct > 100) {
      reply(">>> Hairpin speed range 0~100\r\n");
    } else {
      track_set_hairpin_speed(pct / 100.0f);
      char buf[88];
      snprintf(buf, sizeof(buf), ">>> Hairpin speed set to %d%% (bang-bang forward speed on hairpin turn CH1/CH8)\r\n", pct);
      reply(buf);
    }

  // --- minpwm N (方案A外轮最小前进PWM，防静摩擦堵转停车，0~255) ---
  } else if (strncmp(cmd, "minpwm ", 7) == 0) {
    int v = atoi(cmd + 7);
    if (v < 0 || v > 255) {
      reply(">>> Min move PWM range 0~255\r\n");
    } else {
      track_set_min_move_pwm(v);
      char buf[88];
      snprintf(buf, sizeof(buf), ">>> Min move PWM set to %d (bang-bang outer wheel floor, anti-stall)\r\n", v);
      reply(buf);
    }

  // --- 算法管理单元（见"算法管理单元设计.md"）---
  } else if (strcmp(cmd, "algolist") == 0) {
    track_capture_active();   // 同步激活条目的生效值，保证列出的是最新
    int act = track_algo_active();
    reply(">>> algos (id name base params | *=active):\r\n");
    for (int id = 0; id < ALGO_MAX; id++) {
      if (!track_algo_used(id)) continue;
      AlgoEntry e = track_get_entry(id);
      char extra[72];
      if (e.base == TRACK_ALGO_PID)
        snprintf(extra, sizeof(extra), "kp=%.1f ki=%.1f kd=%.1f", e.params.pid_kp, e.params.pid_ki, e.params.pid_kd);
      else
        snprintf(extra, sizeof(extra), "sharp=%.2f xsharp=%.2f minpwm=%d", e.params.sharp_ratio, e.params.xsharp_ratio, e.params.min_move_pwm);
      char buf[176];
      snprintf(buf, sizeof(buf), "  %c%d %-15s %s speed=%d slew=%.0f %s\r\n",
               id == act ? '*' : ' ', id, e.name, e.base == TRACK_ALGO_PID ? "PID" : "BB",
               e.params.speed, e.params.slew_rate, extra);
      reply(buf);
      // 激活条目：额外打印一行"全部参数"，方便直接查看当前算法的完整调参
      if (id == act) {
        char full[224];
        snprintf(full, sizeof(full),
                 "     ^full: turn=%.2f medium=%.2f sharp=%.2f xsharp=%.2f medspeed=%.2f shpspeed=%.2f hpspeed=%.2f minpwm=%d | pid kp=%.1f ki=%.1f kd=%.1f\r\n",
                 e.params.turn_ratio, e.params.medium_ratio, e.params.sharp_ratio, e.params.xsharp_ratio,
                 e.params.medium_speed, e.params.sharp_speed, e.params.hairpin_speed, e.params.min_move_pwm,
                 e.params.pid_kp, e.params.pid_ki, e.params.pid_kd);
        reply(full);
      }
    }

  } else if (strncmp(cmd, "algouse ", 8) == 0) {
    int id = atoi(cmd + 8);
    if (track_algo_use(id)) {
      AlgoEntry e = track_get_entry(id);
      char buf[80];
      snprintf(buf, sizeof(buf), ">>> Active algo -> id %d '%s' (%s)\r\n", id, e.name, e.base == TRACK_ALGO_PID ? "PID" : "BB");
      reply(buf);
    } else {
      reply(">>> algouse failed: invalid or unused id (see algolist)\r\n");
    }

  } else if (strncmp(cmd, "algonew ", 8) == 0) {
    const char* rest = cmd + 8;
    int base = -1;
    const char* namep = NULL;
    if (strncmp(rest, "BB ", 3) == 0 || strncmp(rest, "bb ", 3) == 0) { base = TRACK_ALGO_BANGBANG; namep = rest + 3; }
    else if (strncmp(rest, "PID ", 4) == 0 || strncmp(rest, "pid ", 4) == 0) { base = TRACK_ALGO_PID; namep = rest + 4; }
    if (base < 0) {
      reply(">>> Usage: algonew BB|PID <name>\r\n");
    } else {
      while (*namep == ' ') namep++;
      if (*namep == '\0') {
        reply(">>> algonew needs a name\r\n");
      } else {
        int id = track_algo_new(base, namep);
        if (id < 0) reply(">>> algonew failed: no free slot (ALGO_MAX reached)\r\n");
        else {
          char buf[96];
          snprintf(buf, sizeof(buf), ">>> Created algo id %d '%s' (%s); 'algouse %d' to switch\r\n",
                   id, namep, base == TRACK_ALGO_PID ? "PID" : "BB", id);
          reply(buf);
        }
      }
    }

  } else if (strncmp(cmd, "algocopy ", 9) == 0) {
    const char* p = cmd + 9;
    int id = atoi(p);
    while (*p == ' ') p++;            // 跳过前导空格
    while (*p && *p != ' ') p++;      // 跳过 id 数字
    while (*p == ' ') p++;            // 跳到名称
    if (*p == '\0') {
      reply(">>> Usage: algocopy N <name>\r\n");
    } else {
      int nid = track_algo_copy(id, p);
      if (nid < 0) reply(">>> algocopy failed: bad id or no free slot\r\n");
      else { char buf[96]; snprintf(buf, sizeof(buf), ">>> Copied id %d -> new id %d '%s'\r\n", id, nid, p); reply(buf); }
    }

  } else if (strncmp(cmd, "algoname ", 9) == 0) {
    const char* p = cmd + 9;
    int id = atoi(p);
    while (*p == ' ') p++;
    while (*p && *p != ' ') p++;
    while (*p == ' ') p++;
    if (*p == '\0') {
      reply(">>> Usage: algoname N <name>\r\n");
    } else if (track_algo_rename(id, p)) {
      char buf[80]; snprintf(buf, sizeof(buf), ">>> Renamed id %d -> '%s'\r\n", id, p); reply(buf);
    } else {
      reply(">>> algoname failed: invalid or unused id\r\n");
    }

  } else if (strncmp(cmd, "algodel ", 8) == 0) {
    int id = atoi(cmd + 8);
    if (track_algo_del(id)) { char buf[64]; snprintf(buf, sizeof(buf), ">>> Deleted algo id %d\r\n", id); reply(buf); }
    else reply(">>> algodel failed: can't delete active/last, or invalid id\r\n");

  // --- algo 0/1 旧命令别名：切到默认条目 id0(bangbang)/id1(pid) ---
  } else if (strcmp(cmd, "algo 0") == 0) {
    track_set_algo(TRACK_ALGO_BANGBANG);
    reply(">>> Active algo -> id 0 (bangbang default) [alias; use 'algouse N']\r\n");
  } else if (strcmp(cmd, "algo 1") == 0) {
    track_set_algo(TRACK_ALGO_PID);
    reply(">>> Active algo -> id 1 (pid default) [alias; use 'algouse N']\r\n");

  // --- pid kp/ki/kd N (float, PID增益) ---
  } else if (strncmp(cmd, "pid kp ", 7) == 0) {
    float v = atof(cmd + 7);
    if (v < 0.0f) {
      reply(">>> PID Kp must be >= 0\r\n");
    } else {
      track_set_pid_kp(v);
      char buf[48];
      snprintf(buf, sizeof(buf), ">>> PID Kp set to %.2f\r\n", v);
      reply(buf);
    }
  } else if (strncmp(cmd, "pid ki ", 7) == 0) {
    float v = atof(cmd + 7);
    if (v < 0.0f) {
      reply(">>> PID Ki must be >= 0\r\n");
    } else {
      track_set_pid_ki(v);
      char buf[48];
      snprintf(buf, sizeof(buf), ">>> PID Ki set to %.2f\r\n", v);
      reply(buf);
    }
  } else if (strncmp(cmd, "pid kd ", 7) == 0) {
    float v = atof(cmd + 7);
    if (v < 0.0f) {
      reply(">>> PID Kd must be >= 0\r\n");
    } else {
      track_set_pid_kd(v);
      char buf[48];
      snprintf(buf, sizeof(buf), ">>> PID Kd set to %.2f\r\n", v);
      reply(buf);
    }

  // --- slewrate N (float, PWM每秒最多变化多少单位，越小越平滑) ---
  } else if (strncmp(cmd, "slewrate ", 9) == 0) {
    float v = atof(cmd + 9);
    if (v < 1.0f) {
      reply(">>> Slew rate must be >= 1\r\n");
    } else {
      track_set_slew_rate(v);
      char buf[48];
      snprintf(buf, sizeof(buf), ">>> Slew rate set to %.1f/s\r\n", v);
      reply(buf);
    }

  // --- lograte N (内存日志采样周期ms，越小越密越吃内存；见"LOG高频采样方案.md") ---
  } else if (strncmp(cmd, "lograte ", 8) == 0) {
    int v = atoi(cmd + 8);
    if (v < 1) {
      reply(">>> Log rate must be >= 1 (ms)\r\n");
    } else {
      track_set_log_interval(v);
      char buf[80];
      snprintf(buf, sizeof(buf), ">>> Log sample interval set to %d ms (smaller=denser, more RAM)\r\n", v);
      reply(buf);
    }

  // --- save ---
  } else if (strcmp(cmd, "save") == 0) {
    config_save();
    char buf[48];
    snprintf(buf, sizeof(buf), ">>> Config saved (speed=%d)\r\n", config_get_speed());
    reply(buf);

  // --- config (print current config as JSON) ---
  } else if (strcmp(cmd, "config") == 0) {
    config_print();

  // --- mem (剩余堆内存+最大可分配连续块+ram_log_module实际分配到的缓冲区大小，见"LOG精简方案.md"4.2节) ---
  } else if (strcmp(cmd, "mem") == 0) {
    char buf[160];
    snprintf(buf, sizeof(buf), ">>> Free heap: %u bytes (min ever: %u), max alloc: %u bytes, ram log buffer: %u bytes\r\n",
             (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
             (unsigned)ESP.getMaxAllocHeap(), (unsigned)ram_log_capacity());
    reply(buf);

  // --- log dump/clear/status ---
  } else if (strcmp(cmd, "log dump") == 0) {
    ram_log_dump();  // 直接从内存打印，不经过flash（见用户反馈，flash落盘先停用）
  } else if (strcmp(cmd, "log clear") == 0) {
    print_file_clear();
    reply(">>> Log file cleared\r\n");
  } else if (strcmp(cmd, "log status") == 0) {
    char buf[96];
    snprintf(buf, sizeof(buf), ">>> File log %s, capacity %ld bytes (ring buffer), next_id=%lu%s\r\n",
             print_file_enabled() ? "ON" : "OFF", print_file_size(), print_file_next_id(),
             print_file_wrapped() ? ", 已绕回过(最老的记录已被覆盖)" : ", 未绕回(尚未写满一圈)");
    reply(buf);
  } else if (strcmp(cmd, "log format") == 0) {
    print_log_legend();

  // --- threshold CH VALUE (8路方案：CH1~CH8，见"8路传感器方案.md") ---
  } else if (strncmp(cmd, "threshold ", 10) == 0) {
    int ch, value;
    if (sscanf(cmd + 10, "%d %d", &ch, &value) != 2 || ch < 1 || ch > 8) {
      reply(">>> Usage: threshold CH VALUE (CH=1~8)\r\n");
    } else {
      sensor_set_threshold(ch - 1, value);
      char buf[64];
      snprintf(buf, sizeof(buf), ">>> CH%d threshold set to %d (memory only, use save to persist)\r\n",
               ch, value);
      reply(buf);
    }

  // --- whiteref/blackref CH VALUE (PID模拟量加权用的白/黑参考值，CH=1~8) ---
  } else if (strncmp(cmd, "whiteref ", 9) == 0) {
    int ch, value;
    if (sscanf(cmd + 9, "%d %d", &ch, &value) != 2 || ch < 1 || ch > 8) {
      reply(">>> Usage: whiteref CH VALUE (CH=1~8)\r\n");
    } else {
      sensor_set_white_ref(ch - 1, value);
      char buf[64];
      snprintf(buf, sizeof(buf), ">>> CH%d white ref set to %d (memory only, use save to persist)\r\n",
               ch, value);
      reply(buf);
    }
  } else if (strncmp(cmd, "blackref ", 9) == 0) {
    int ch, value;
    if (sscanf(cmd + 9, "%d %d", &ch, &value) != 2 || ch < 1 || ch > 8) {
      reply(">>> Usage: blackref CH VALUE (CH=1~8)\r\n");
    } else {
      sensor_set_black_ref(ch - 1, value);
      char buf[64];
      snprintf(buf, sizeof(buf), ">>> CH%d black ref set to %d (memory only, use save to persist)\r\n",
               ch, value);
      reply(buf);
    }

  // --- go N ---
  } else if (strncmp(cmd, "go ", 3) == 0) {
    int sec = atoi(cmd + 3);
    if (sec <= 0 || sec > 60) {
      reply(">>> Time range 1~60 sec\r\n");
    } else {
      int pwm = motor_level_to_pwm(config_get_speed());
      char buf[64];
      snprintf(buf, sizeof(buf), ">>> Forward %d sec, speed %d (PWM %d)\r\n",
               sec, config_get_speed(), pwm);
      reply(buf);
      motor_set(pwm, pwm);
      delay((unsigned long)sec * 1000);
      motor_stop();
      reply(">>> Stopped\r\n");
    }

  // --- back N ---
  } else if (strncmp(cmd, "back ", 5) == 0) {
    int sec = atoi(cmd + 5);
    if (sec <= 0 || sec > 60) {
      reply(">>> Time range 1~60 sec\r\n");
    } else {
      int pwm = motor_level_to_pwm(config_get_speed());
      char buf[64];
      snprintf(buf, sizeof(buf), ">>> Backward %d sec, speed %d (PWM %d)\r\n",
               sec, config_get_speed(), pwm);
      reply(buf);
      motor_set(-pwm, -pwm);
      delay((unsigned long)sec * 1000);
      motor_stop();
      reply(">>> Stopped\r\n");
    }

  // --- spin left/right N (原地转圈，测试电机扭矩) ---
  } else if (strncmp(cmd, "spin left ", 10) == 0) {
    int sec = atoi(cmd + 10);
    if (sec <= 0 || sec > 60) {
      reply(">>> Time range 1~60 sec\r\n");
    } else {
      int pwm = motor_level_to_pwm(config_get_speed());
      char buf[64];
      snprintf(buf, sizeof(buf), ">>> Spin left %d sec, speed %d (PWM %d)\r\n",
               sec, config_get_speed(), pwm);
      reply(buf);
      motor_set(-pwm, pwm);
      delay((unsigned long)sec * 1000);
      motor_stop();
      reply(">>> Stopped\r\n");
    }

  } else if (strncmp(cmd, "spin right ", 11) == 0) {
    int sec = atoi(cmd + 11);
    if (sec <= 0 || sec > 60) {
      reply(">>> Time range 1~60 sec\r\n");
    } else {
      int pwm = motor_level_to_pwm(config_get_speed());
      char buf[64];
      snprintf(buf, sizeof(buf), ">>> Spin right %d sec, speed %d (PWM %d)\r\n",
               sec, config_get_speed(), pwm);
      reply(buf);
      motor_set(pwm, -pwm);
      delay((unsigned long)sec * 1000);
      motor_stop();
      reply(">>> Stopped\r\n");
    }

  // --- help ---
  } else if (strcmp(cmd, "help") == 0) {
    reply(">>> Available commands:\r\n");
    reply("    print on/off         USB+BT data stream on/off\r\n");
    reply("    print usb on/off     USB data stream only\r\n");
    reply("    print bt  on/off     BT  data stream only\r\n");
    reply("    print file on/off    log data stream to /track.log.0~3 flash files, ring buffer w/ #ID (for when BT drops)\r\n");
    reply("    go N                 forward N sec (1~60)\r\n");
    reply("    back N               backward N sec (1~60)\r\n");
    reply("    spin left N          spin in place left N sec (1~60)\r\n");
    reply("    spin right N         spin in place right N sec (1~60)\r\n");
    reply("    stop                 stop motor immediately (also cancels track)\r\n");
    reply("    track on/off         start/stop autonomous line tracking (log: RAM-only during run until buffer nearly full, backed up to flash on off)\r\n");
    reply("    speed N              speed level (1~40, default 12, until changed)\r\n");
    reply("    turnspeed N          outer wheel ratio on sharp/hairpin turn (0~100%, default 65)\r\n");
    reply("    mediumratio N        inner wheel speed on medium turn CH3/CH6 (0~100%, default 35)\r\n");
    reply("    sharpratio N         inner wheel reverse ratio on sharp turn CH2/CH7 (0~100%, default 30)\r\n");
    reply("    xsharpratio N        inner wheel reverse ratio on hairpin turn CH1/CH8 (0~100%, default 60)\r\n");
    reply("    mediumspeed N        [bangbang] forward speed on medium turn CH3/CH6 (0~100%, default 85)\r\n");
    reply("    sharpspeed N         [bangbang] forward speed on sharp turn CH2/CH7 (0~100%, default 65)\r\n");
    reply("    hairpinspeed N       [bangbang] forward speed on hairpin turn CH1/CH8 (0~100%, default 50)\r\n");
    reply("    minpwm N             [bangbang] outer wheel min forward PWM, anti-stall floor (0~255, default 70)\r\n");
    reply("  -- algorithm manager (named param profiles, base=bangbang/pid) --\r\n");
    reply("    algolist             list algo profiles (id name base params, *=active)\r\n");
    reply("    algouse N            switch active profile to id N\r\n");
    reply("    algonew BB|PID name  create a new profile (default params)\r\n");
    reply("    algocopy N name      duplicate profile N to a new slot\r\n");
    reply("    algoname N name      rename profile N\r\n");
    reply("    algodel N            delete profile N (not active/last)\r\n");
    reply("    algo 0/1             alias: switch to id 0(bangbang)/1(pid) default profile\r\n");
    reply("    pid kp N             PID proportional gain (float, default 40.0)\r\n");
    reply("    pid ki N             PID integral gain (float, default 0.0)\r\n");
    reply("    pid kd N             PID derivative gain (float, default 5.0)\r\n");
    reply("    slewrate N           max PWM change per second (float, default 800, smaller = smoother/less zigzag)\r\n");
    reply("    lograte N            RAM log sample interval ms (default 10; 1=near per-loop/~1lap, 500=save RAM)\r\n");
    reply("    whiteref CH VALUE    set CHx (1~8) white reference (analog weighting for PID), memory only\r\n");
    reply("    blackref CH VALUE    set CHx (1~8) black reference (analog weighting for PID), memory only\r\n");
    reply("    save                 save speed+turnspeed+mediumratio+sharpratio+xsharpratio+medium/sharp/hairpin speed+minpwm+algo+pid gains+slewrate+file log on/off+thresholds+white/black refs to flash (/config.json)\r\n");
    reply("    config               print current config as JSON\r\n");
    reply("    mem                  show free heap + ram log buffer size (auto-sized at boot, see ram_log_auto_init())\r\n");
    reply("    log dump             print current RAM log buffer over Serial (most recent track on run, no flash involved)\r\n");
    reply("    log clear            wipe all /track.log.* segments, #ID resets to 0\r\n");
    reply("    log status           show whether file log is on, capacity, next #ID, wrapped y/n\r\n");
    reply("    log format           print event/heartbeat line format + modeCode legend\r\n");
    reply("    threshold CH VALUE   set CHx (1~8) threshold, memory only\r\n");
    reply("    help                 show this help\r\n");

  } else {
    reply(">>> Unknown command, type help for command list\r\n");
  }
}

// Serial 行读取（带回显）
static char _serial_buf[64];
static int  _serial_len = 0;

static bool serial_poll_line(char* buf, int maxlen) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (_serial_len == 0) continue;
      Serial.print("\r\n");
      _serial_buf[_serial_len] = '\0';
      strncpy(buf, _serial_buf, maxlen - 1);
      buf[maxlen - 1] = '\0';
      _serial_len = 0;
      return true;
    } else if (c == 0x08 || c == 0x7F) {
      if (_serial_len > 0) {
        _serial_len--;
        Serial.print("\x08 \x08");
      }
    } else if (_serial_len < (int)sizeof(_serial_buf) - 1) {
      _serial_buf[_serial_len++] = c;
      Serial.write(c);
    }
  }
  return false;
}

void cmd_begin() {
  show_prompt();
}

void cmd_poll() {
  char line[64];
  if (bt_poll_line(line, sizeof(line))) {
    handle_command(line);
    show_prompt();
  }
  if (serial_poll_line(line, sizeof(line))) {
    handle_command(line);
    show_prompt();
  }
}
