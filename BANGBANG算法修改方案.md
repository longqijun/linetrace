# BANGBANG 算法修改方案

> 目标：让中间两路（CH4/CH5）压线时小车保持直行、不改变轮子状态；转向档位重新按对称三档分组。
> 依据：log13 分析发现小车跑直线时从不进 STRAIGHT，居中读数反而触发转向，导致自激振荡丢线。

## 一、需求

1. **CH4 / CH5 压线**（不管只压一路还是两路都压）→ **直行**，左右轮都给 base，不做任何转向修正。
2. 转向档位按对称三档分组（数字为传感器通道号）：
   - **CH3 与 CH6 = 一档**（离中心第二近，最缓的转向）
   - **CH2 与 CH7 = 一档**（第三近，中等）
   - **CH1 与 CH8 = 一档**（最外，最急）

## 二、现状（4 档）

物理左→右排列：`CH8 CH7 CH6 CH5 | CH4 CH3 CH2 CH1`

| 传感器 | 现档位 | 内轮比例 | modeCode |
|---|---|---|---|
| CH5 / CH4 | mild 缓转 | `TURN_RATIO_MILD=0.7` | 1 / 2 |
| CH6 / CH3 | medium 中转 | `_medium_ratio=0.35`（不反转） | 3 / 4 |
| CH7 / CH2 | sharp 急转 | `_sharp_ratio=-0.3`（反转）+外轮降速 | 5 / 6 |
| CH8 / CH1 | xsharp 发卡 | `_xsharp_ratio=-0.6`（反转）+外轮降速 | 7 / 8 |

问题：CH5/CH4 单独压线也会被判成缓转（mode 1/2）不断微调，加上外层反转档，直线上越摆越大。

## 三、目标（中心直行 + 三档）

| 传感器 | 新档位 | 处理 | modeCode |
|---|---|---|---|
| **CH5 / CH4** | **直行** | 左右轮 = base，不转向、不更新 `_last_dir` | **0（STRAIGHT）** |
| CH6 / CH3 | 一档（原 medium） | 内轮 `_medium_ratio`，不反转 | 3 / 4 |
| CH7 / CH2 | 二档（原 sharp） | 内轮 `_sharp_ratio` 反转 + 外轮降速 | 5 / 6 |
| CH8 / CH1 | 三档（原 xsharp） | 内轮 `_xsharp_ratio` 反转 + 外轮降速 | 7 / 8 |

**关键点**：你要的三档分组和现有的 medium/sharp/xsharp 三档**完全一一对应**，所以档位逻辑和三个可调参数（`mediumratio`/`sharpratio`/`xsharpratio`）都不用动，只需**删掉 mild 这一档**。删掉后 CH4/CH5 单独压线时不会命中任何转向分支，自然落到默认的 `pwm_l=pwm_r=base`、`mode_code='0'`，正好就是直行。

判定顺序仍是**由外到内**（先看 CH8/CH1，再 CH7/CH2，再 CH6/CH3），命中外层就不看内层；十字/宽线（左右两半同时压线）仍走 mode 9 直行穿过。这两条不变。

## 四、具体代码改动（track_module.cpp）

改动集中在 `track_update()` 的 bangbang 分支和几处定义/注释，逻辑改动很小：

1. **删掉 mild 档定义**：移除 `#define TURN_RATIO_MILD 0.7f`（第 16-17 行）。
2. **删掉 mild 布尔量与计算**：移除 `bool mild_l/mild_r`（第 280-281 行）、`int mild_turn = ...`（第 295 行）。
3. **删掉两个 mild 分支**：移除 `else if (mild_l) { ... mode_code='1'; }`（第 380-383 行）和 `else if (mild_r) { ... mode_code='2'; }`（第 398-401 行）。删掉后 bangbang 分支保留 6 个转向分支（xsharp_l/sharp_l/medium_l/xsharp_r/sharp_r/medium_r），CH4/CH5 落入默认直行。
4. **更新注释**：第 12-13 行、第 56 行、track_module.h 第 5 行里描述四级档位的说明，改成"CH4/CH5 直行 + 三档转向"。

**不改动**：medium/sharp/xsharp 三档的比例参数、外轮降速 `_turn_outer_ratio`、丢线找线逻辑、cross 判定、PID 分支、限速、日志格式。

## 五、副作用与注意事项

1. **一档变"硬"了**：以前第一级修正是 mild（内轮 0.7，很缓），现在删掉后，最轻的修正直接变成原 medium（内轮 `_medium_ratio=0.35`）。也就是说线一旦从中心两路挪到 CH3/CH6，修正力度比以前大。
   - 如果实车觉得一档太猛，可运行时用 `mediumratio` 命令把它调大（更缓，如 0.5~0.7），或后续把 `_medium_ratio` 默认值上调。
   - 这是"中心区加宽为直行死区"的必然代价，属于预期内。
2. **mode 1 / 2 不再出现**：日志里不会再有 LEFT(mild)/RIGHT(mild)。建议同步更新 `print_module.cpp` 第 157 行的 legend，把 `1=LEFT(mild) 2=RIGHT(mild)` 标注为未使用（或删除），避免看 log 时误解。
3. **内部变量名**：medium/sharp/xsharp 这些名字保留不改（改名是纯改动风险，无功能收益），只更新注释说明它们现在对应"一档/二档/三档"。
4. **直行死区变宽**：CH4+CH5 覆盖的物理宽度内都算直行，小抖动被吸收，这正是解决 log13 振荡的核心；但若线细、传感器间距大，中心两路可能出现"都不压线"的瞬间（落入 mode 9 或短暂丢线找回），属正常。

## 六、待你确认

- [ ] 一档（CH3/CH6）默认力度：保持现在的 `_medium_ratio=0.35`，还是先调缓一点（如 0.5）？
- [ ] mode 1/2 的 legend：删除还是保留标注为未使用？
- [ ] 是否需要我同步把 `print_module.cpp` 的 legend 和相关文档一起改掉？

确认后我再改代码。
