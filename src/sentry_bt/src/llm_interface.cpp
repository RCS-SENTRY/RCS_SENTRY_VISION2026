#include "llm_interface.hpp"

LLMInterface::LLMInterface() = default;

void LLMInterface::Update(RobotContext& ctx)
{
    std::lock_guard<std::mutex> lock(ctx.mtx);
    const bool raw_need_supply = (ctx.ammo_17 < 80) || (ctx.hp < (ctx.hp_max / 3));

    ctx.llm_advice = {};
    ctx.llm_advice.expire_time_ms = (ctx.frame_index * 50U) + 500U;

    if (!ctx.referee_link_fresh)
    {
        ctx.llm_advice.reason = "输入链路超时，暂停外部建议。";
        return;
    }

    if (ctx.is_dead)
    {
        ctx.llm_advice.reason = "处于等待复活阶段，暂不生成外部建议。";
        return;
    }

    if (ctx.enemy_in_view && ctx.enemy_confidence > 0.85f)
    {
        ctx.llm_advice.valid = true;
        ctx.llm_advice.tactical_state = TacticalState::ENGAGE;
        ctx.llm_advice.goal_id = "CURRENT_HOLD";
        ctx.llm_advice.posture_preference = Posture::ATTACK;
        ctx.llm_advice.fire_policy = FirePolicy::AGGRESSIVE;
        ctx.llm_advice.spin_preference = (ctx.enemy_distance_m < 3.5f) ? SpinMode::ON
                                                                       : SpinMode::OFF;
        ctx.llm_advice.supercap_preference = SupercapMode::BURST;
        ctx.llm_advice.reason = "检测到高置信度敌方目标，建议优先直接交战。";
        ctx.llm_advice.confidence = 0.92f;
        return;
    }

    if (!ctx.enemy_in_view && !ctx.on_base && !ctx.on_highground && !raw_need_supply)
    {
        ctx.llm_advice.valid = true;
        ctx.llm_advice.tactical_state = TacticalState::REPOSITION;
        ctx.llm_advice.goal_id = "MID_CROSS";
        ctx.llm_advice.posture_preference = Posture::MOVE;
        ctx.llm_advice.fire_policy = FirePolicy::CONSERVATIVE;
        ctx.llm_advice.spin_preference = SpinMode::OFF;
        ctx.llm_advice.supercap_preference = SupercapMode::KEEP;
        ctx.llm_advice.reason = "当前没有可见目标，建议转向更好的观察与射击通道。";
        ctx.llm_advice.confidence = 0.78f;
        return;
    }

    if (ctx.on_base || ctx.on_fortress || ctx.on_outpost)
    {
        ctx.llm_advice.valid = true;
        ctx.llm_advice.tactical_state = TacticalState::HOLD;
        ctx.llm_advice.goal_id =
            ctx.on_base ? "BASE_HOLD" : (ctx.on_fortress ? "FORTRESS_HOLD" : "OUTPOST_HOLD");
        ctx.llm_advice.posture_preference = Posture::DEFENSE;
        ctx.llm_advice.fire_policy = FirePolicy::NORMAL;
        ctx.llm_advice.spin_preference = SpinMode::OFF;
        ctx.llm_advice.supercap_preference = SupercapMode::OFF;
        ctx.llm_advice.reason = "当前位置具备战略价值，建议继续坚守。";
        ctx.llm_advice.confidence = 0.70f;
        return;
    }

    ctx.llm_advice.valid = true;
    ctx.llm_advice.tactical_state = TacticalState::SEARCH;
    ctx.llm_advice.goal_id = "SEARCH_AREA_A";
    ctx.llm_advice.posture_preference = Posture::MOVE;
    ctx.llm_advice.fire_policy = FirePolicy::CONSERVATIVE;
    ctx.llm_advice.spin_preference = SpinMode::ON;
    ctx.llm_advice.supercap_preference = SupercapMode::OFF;
    ctx.llm_advice.reason = "没有更强的先验信号，建议继续搜索侦察。";
    ctx.llm_advice.confidence = 0.55f;
}
