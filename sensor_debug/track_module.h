#pragma once

// 自动巡线，两种算法并列，运行时可切换（默认BANGBANG）：
//   TRACK_ALGO_BANGBANG：中心直行+对称三档差速bang-bang（8路传感器，见"8路传感器方案.md"）：
//     CH4/CH5直行(不改变轮速)，CH3/CH6一档中转(medium)，CH2/CH7二档急转(sharp)，CH1/CH8三档发卡弯(hairpin)
//   TRACK_ALGO_PID：连续加权位置(sensor_position_from)作误差输入的PID闭环差速
// 丢线延续方向找线、十字路口/宽线直行穿过这两个行为，两种算法共用同一套判定，不受算法切换影响

#define TRACK_ALGO_BANGBANG 0
#define TRACK_ALGO_PID       1

// ===== 算法管理单元（见"算法管理单元设计.md"）=====
// 一个"算法条目(AlgoEntry)" = id + 名称 + 控制律(base) + 一整套参数(AlgoProfile)。
// 管理单元持有多个条目(可多个共用同一 base，比如好几套 bangbang 调参)，一个"激活条目"决定当前跑什么。
// 硬件标定(阈值/白黑参考/file_log)由 config_module 单独共享，不进条目。
#define ALGO_MAX       8    // 最多条目数
#define ALGO_NAME_LEN  16   // 名称最长15字符+结束符(UTF-8下约5汉字)

// 参数集：一个 struct 装下所有 base 的字段，某 base 用不到的字段随存不影响别的 base。
typedef struct {
  int   speed;
  float slew_rate;
  float turn_ratio, medium_ratio, sharp_ratio, xsharp_ratio;  // bangbang 转向比例
  float medium_speed, sharp_speed, hairpin_speed;             // bangbang 弯道分档降速(方案A)
  int   min_move_pwm;                                          // bangbang 外轮最小前进PWM
  float pid_kp, pid_ki, pid_kd;                                // pid 增益
} AlgoProfile;

typedef struct {
  bool  used;                 // 槽位是否启用(支持删除留空)
  char  name[ALGO_NAME_LEN];  // 算法名 algoname
  int   base;                 // 控制律 TRACK_ALGO_BANGBANG / TRACK_ALGO_PID
  AlgoProfile params;         // 该条目的参数集
} AlgoEntry;

// ---- 算法管理单元 API ----
int  track_algo_active();                        // 当前激活条目 id
bool track_algo_use(int id);                     // 切换激活条目(热切换,清PID历史)；无效/未启用返回false
int  track_algo_new(int base, const char* name); // 新建条目(默认参数)，返回新id；满了返回-1
int  track_algo_copy(int id, const char* name);  // 复制条目到空槽，返回新id；失败-1
bool track_algo_rename(int id, const char* name);// 重命名；失败false
bool track_algo_del(int id);                     // 删除(不能删激活/最后一个)；失败false
bool track_algo_used(int id);                    // 槽位是否启用
AlgoEntry track_get_entry(int id);               // 读某条目(供config/algolist)
AlgoProfile track_default_params();              // 一套默认参数(供config新建条目兜底)
void track_capture_active();                     // 把当前生效值回存进激活条目(save/print/list前调)
void track_load_entries(const AlgoEntry* arr, int active_id); // config加载:整体灌入并应用激活条目

void track_begin();          // 初始化，默认关闭
void track_set(bool on);     // 开启/关闭巡线模式（关闭时会停止电机）
bool track_is_on();          // 是否处于巡线模式
void track_update();         // 每次loop调用，仅开启时生效，非阻塞

float track_get_turn_ratio();          // 获取急弯外轮速度比例(0.0~1.0)，占base的百分比
void  track_set_turn_ratio(float ratio); // 设置急弯外轮速度比例（仅内存，供config_module持久化用）

float track_get_sharp_ratio();          // 获取急弯内轮反转比例(-1.0~0.0)，占base的百分比
void  track_set_sharp_ratio(float ratio); // 设置急弯内轮反转比例（仅内存，供config_module持久化用）

float track_get_medium_ratio();          // 获取一档中转(CH3/CH6)内轮速度比例(0.0~1.0)
void  track_set_medium_ratio(float ratio); // 设置中转内轮速度比例（仅内存，供config_module持久化用）

float track_get_xsharp_ratio();          // 获取发卡弯(CH1/CH8)内轮反转比例(-1.0~0.0)，比sharp更激进
void  track_set_xsharp_ratio(float ratio); // 设置发卡弯内轮反转比例（仅内存，供config_module持久化用）

// 方案A（弯道降速，仅bang-bang）：各档前进速度系数(0.0~1.0)，越小进弯越慢，PID不受影响
float track_get_medium_speed();           // MEDIUM档(CH3/CH6)前进速度系数
void  track_set_medium_speed(float ratio);
float track_get_sharp_speed();            // SHARP档(CH2/CH7)前进速度系数
void  track_set_sharp_speed(float ratio);
float track_get_hairpin_speed();          // HAIRPIN档(CH1/CH8)前进速度系数
void  track_set_hairpin_speed(float ratio);
int   track_get_min_move_pwm();           // 外轮最小前进PWM(防静摩擦堵转失速)，0~255
void  track_set_min_move_pwm(int pwm);

int  track_get_algo();       // 获取当前激活条目的控制律（TRACK_ALGO_BANGBANG/TRACK_ALGO_PID）
void track_set_algo(int algo); // 旧命令别名：切到 id0(BANGBANG)/id1(PID) 默认条目，等价 track_algo_use

float track_get_pid_kp();          // 获取PID比例增益
void  track_set_pid_kp(float kp);  // 设置PID比例增益（>=0，仅内存，供config_module持久化用）
float track_get_pid_ki();          // 获取PID积分增益
void  track_set_pid_ki(float ki);  // 设置PID积分增益（>=0，仅内存，供config_module持久化用）
float track_get_pid_kd();          // 获取PID微分增益
void  track_set_pid_kd(float kd);  // 设置PID微分增益（>=0，仅内存，供config_module持久化用）

float track_get_slew_rate();          // 获取PWM限速值(单位/秒)，限制下发给电机的pwm_l/r每秒最多变化多少
void  track_set_slew_rate(float rate); // 设置PWM限速值（>0，仅内存，供config_module持久化用）

int  track_get_log_interval();        // 内存日志采样周期(ms)，越小越密(近连续轨迹)，越大越省内存
void track_set_log_interval(int ms);  // 设置采样周期（>=1，仅内存，供config_module持久化用；lograte命令）
