#ifndef SENTRY_DECISION_PROTOCOL_H
#define SENTRY_DECISION_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 哨兵决策下发协议草案
 *
 * 设计目标：
 * 1. 承载当前 BT executor 的最终控制输出；
 * 2. 承载规则树 / 裁判动作树产生的一次性动作；
 * 3. 作为“上位决策 -> 下位执行/打包”的中间层协议，不包含串口包头、CRC 等外层封装。
 *
 * 重要说明：
 * - 这里的 enum 只用于提供符号名，真正进结构体的字段统一使用定长整数类型。
 * - 不要把 C/C++ 原始 enum 下标直接当作线上协议值；请按本头文件定义做显式映射。
 */

#define SENTRY_DECISION_PROTOCOL_VERSION 1u

#if defined(__GNUC__)
#define SENTRY_PACKED __attribute__((packed))
#else
#define SENTRY_PACKED
#endif

/*
 * 目标点 ID。
 * 当前取值来自行为树中的候选目标点与 executor 兜底目标。
 */
typedef enum
{
    SENTRY_GOAL_ID_INVALID = 0,
    SENTRY_GOAL_ID_SAFE_HOLD = 1,
    SENTRY_GOAL_ID_WAIT_REVIVE = 2,
    SENTRY_GOAL_ID_SAFE_RETREAT_A = 3,
    SENTRY_GOAL_ID_SAFE_RETREAT_B = 4,
    SENTRY_GOAL_ID_SUPPLY_LEFT = 5,
    SENTRY_GOAL_ID_SUPPLY_RIGHT = 6,
    SENTRY_GOAL_ID_FORTRESS_HOLD = 7,
    SENTRY_GOAL_ID_OUTPOST_HOLD = 8,
    SENTRY_GOAL_ID_COMBAT_KITE_A = 9,
    SENTRY_GOAL_ID_COMBAT_HOLD_A = 10,
    SENTRY_GOAL_ID_MID_PRESSURE = 11,
    SENTRY_GOAL_ID_HIGHGROUND_PEEK = 12,
    SENTRY_GOAL_ID_COMBAT_PUSH_A = 13,
    SENTRY_GOAL_ID_SEARCH_AREA_A = 14,
    SENTRY_GOAL_ID_SEARCH_AREA_B = 15,
    SENTRY_GOAL_ID_HIGHGROUND_SCAN = 16,
    SENTRY_GOAL_ID_HIGHGROUND_CENTER = 17,
    SENTRY_GOAL_ID_MID_CROSS = 18
} sentry_goal_id_e;

/*
 * 高层战术态。
 * 该字段用于表达“本轮为什么进入这类行为子树”。
 */
typedef enum
{
    SENTRY_TACTICAL_STATE_IDLE = 0,
    SENTRY_TACTICAL_STATE_HOLD = 1,
    SENTRY_TACTICAL_STATE_ENGAGE = 2,
    SENTRY_TACTICAL_STATE_RETREAT = 3,
    SENTRY_TACTICAL_STATE_RESUPPLY = 4,
    SENTRY_TACTICAL_STATE_SEARCH = 5,
    SENTRY_TACTICAL_STATE_REPOSITION = 6,
    SENTRY_TACTICAL_STATE_RUNE_SUPPORT = 7
} sentry_tactical_state_e;

/*
 * 姿态协议值。
 * 这里直接对齐裁判系统 0x0120 的姿态编码：
 * 1=进攻，2=防御，3=移动。
 */
typedef enum
{
    SENTRY_POSTURE_NONE = 0,
    SENTRY_POSTURE_ATTACK = 1,
    SENTRY_POSTURE_DEFENSE = 2,
    SENTRY_POSTURE_MOVE = 3
} sentry_posture_e;

/*
 * 开火策略。
 * 数值越大，表示越激进。
 */
typedef enum
{
    SENTRY_FIRE_POLICY_HOLD_FIRE = 0,
    SENTRY_FIRE_POLICY_CONSERVATIVE = 1,
    SENTRY_FIRE_POLICY_NORMAL = 2,
    SENTRY_FIRE_POLICY_AGGRESSIVE = 3
} sentry_fire_policy_e;

typedef enum
{
    SENTRY_SPIN_MODE_OFF = 0,
    SENTRY_SPIN_MODE_ON = 1
} sentry_spin_mode_e;

typedef enum
{
    SENTRY_SUPERCAP_MODE_OFF = 0,
    SENTRY_SUPERCAP_MODE_KEEP = 1,
    SENTRY_SUPERCAP_MODE_BURST = 2
} sentry_supercap_mode_e;

/*
 * 当前规则树抢占到的动作类型。
 * 该字段更偏“解释量/分支量”，便于下位机了解当前为何偏离常规战术输出。
 */
typedef enum
{
    SENTRY_RULE_ACTION_NONE = 0,
    SENTRY_RULE_ACTION_EXCHANGE_AMMO_AT_POINT = 1,
    SENTRY_RULE_ACTION_REMOTE_AMMO = 2,
    SENTRY_RULE_ACTION_REMOTE_HP = 3,
    SENTRY_RULE_ACTION_CONFIRM_REVIVE = 4,
    SENTRY_RULE_ACTION_INSTANT_REVIVE = 5,
    SENTRY_RULE_ACTION_SWITCH_POSTURE = 6,
    SENTRY_RULE_ACTION_CHANGE_POSTURE = 6,
    SENTRY_RULE_ACTION_ACTIVATE_ENERGY = 7,
    SENTRY_RULE_ACTION_CLAIM_PERIODIC_AMMO = 0
} sentry_rule_action_type_e;

/*
 * 复活相关动作。
 * 这是对 revive 子树的抽象收敛，不直接等同于裁判协议位图。
 */
typedef enum
{
    SENTRY_REVIVE_CMD_NONE = 0,
    SENTRY_REVIVE_CMD_CONFIRM_FREE = 1,
    SENTRY_REVIVE_CMD_CONFIRM_IMMEDIATE = 2,
    SENTRY_REVIVE_CMD_WAIT = 3
} sentry_revive_cmd_e;

/*
 * 持续控制量：
 * 每个决策周期都可以整体覆盖一次，适合直接下发给运动/射击/功率控制模块。
 */
typedef struct SENTRY_PACKED
{
    /* 目标点 ID，取值见 sentry_goal_id_e。 */
    uint8_t goal_id;

    /* 当前高层战术态，取值见 sentry_tactical_state_e。 */
    uint8_t tactical_state;

    /* executor 收口后的最终姿态目标，取值见 sentry_posture_e。 */
    uint8_t posture;

    /* executor 收口后的最终开火策略，取值见 sentry_fire_policy_e。 */
    uint8_t fire_policy;

    /* 小陀螺模式，取值见 sentry_spin_mode_e。 */
    uint8_t spin_mode;

    /* 超级电容使用模式，取值见 sentry_supercap_mode_e。 */
    uint8_t supercap_mode;

    /* 当前抢占的规则动作类型，取值见 sentry_rule_action_type_e。 */
    uint8_t rule_action_type;
} sentry_control_cmd_t;

/*
 * 规则/裁判动作量：
 * 这部分字段不是简单“覆盖”，而是混合了保持量、累计量、增量和脉冲量。
 *
 * 字段语义约定：
 * - revive_cmd：状态量，可保持到下一次状态变化。
 * - ammo_exchange_target_total：累计目标值，必须单调不减。
 * - remote_*_req_inc：增量值，推荐单周期只发送 0 或 1。
 * - posture_cmd_referee：向裁判侧发送的姿态切换指令，0 表示本周期不下发。
 * - activate_energy_confirm / claim_periodic_ammo：脉冲量，1 表示本周期触发一次。
 */
typedef struct SENTRY_PACKED
{
    /*
     * 哨兵兑换允许发弹量的累计目标值。
     * 直接对齐裁判系统 0x0120 bit[2:12] 的“将要兑换的发弹量值”。
     * 单位：发。
     */
    uint16_t ammo_exchange_target_total;

    /* 复活动作，取值见 sentry_revive_cmd_e。 */
    uint8_t revive_cmd;

    /*
     * 本周期新增的“远程兑换发弹量请求次数”。
     * 推荐值：0 或 1。下位机若负责组 0x0120，应将其累加到单调计数器中。
     */
    uint8_t remote_ammo_req_inc;

    /*
     * 本周期新增的“远程兑换血量请求次数”。
     * 推荐值：0 或 1。下位机若负责组 0x0120，应将其累加到单调计数器中。
     */
    uint8_t remote_hp_req_inc;

    /*
     * 向裁判系统发送的姿态切换请求。
     * 0=本周期不切换，其余取值见 sentry_posture_e。
     */
    uint8_t posture_cmd_referee;

    /*
     * 是否确认让能量机关进入“正在激活”状态。
     * 0=不触发，1=本周期触发一次。
     */
    uint8_t activate_energy_confirm;

    /*
     * 是否领取周期补弹奖励。
     * 这是本项目为下位机预留的本地扩展位，不直接对应裁判 0x0120。
     * 0=不触发，1=本周期触发一次。
     */
    uint8_t claim_periodic_ammo;
} sentry_referee_action_cmd_t;

/*
 * 推荐的完整下发载荷。
 * 若链路带宽或模块拆分有要求，也可把 control / referee 两个子结构拆成两帧发送。
 */
typedef struct SENTRY_PACKED
{
    /* 版本号，便于后续协议扩展。 */
    uint8_t protocol_version;

    /* 持续控制量。 */
    sentry_control_cmd_t control;

    /* 规则/裁判动作量。 */
    sentry_referee_action_cmd_t referee;
} sentry_decision_payload_t;

#ifdef __cplusplus
}
#endif

#endif /* SENTRY_DECISION_PROTOCOL_H */
