from typing import Tuple

import numpy as np

from auto_mst_rl_xuance.envs.sim_py.ce.tools.agent_utils import (  # Agent 仍在 agent_utils 中
    RuntimeState,  # 从 GlobalState 更改而来
    get_candidate_neighbors,
    get_topology_neighbors,
)
from auto_mst_rl_xuance.envs.sim_py.ce.tools.utils import (
    angle_of_vectors,
    calculate_heading_entropy,
    unitvel,
)


def swarm_alg_ce(
    runtime_state: RuntimeState,
) -> Tuple[np.ndarray, np.ndarray, RuntimeState]:
    """
    追逃场景的群体算法

    Args:
        runtime_state: 运行时状态

    Returns:
        Tuple: (期望转向角, 期望速度, 更新后的运行时状态)
    """
    config = runtime_state.config
    metrics = runtime_state.metrics
    actors = runtime_state.actors
    # 假设 des_turn_angle 和 des_speed 应根据 max_id 确定大小
    num_agents_for_arrays = runtime_state.max_id

    # 普通个体的机动过程
    # 在每次循环的末尾记录激活状态
    activated_this_step = []  # 存储本步骤激活的个体ID
    activated_src_this_step = []  # 存储本步骤激活的个体的源ID

    # ----- Self-Propelled: 个体自驱动方向 -----
    # 期望速度/方向数组，索引从 0 到 num_agents_for_arrays-1
    i_vel_prop = np.zeros((num_agents_for_arrays, 2))
    # 只为猎物计算自驱动，捕食者有独立的行为逻辑
    for agent_id in runtime_state.prey_list:
        # 自驱作用力：维持原来运动方向的作用
        # agent_id 是从1开始的，数组索引是从0开始的
        i_vel_prop[agent_id - 1] = unitvel(actors[agent_id].vel)

    # ----- AGGREGATION: 个体速度协同 -----
    dir_vel_align = np.zeros((num_agents_for_arrays, 2))
    # 只为猎物计算速度协同，捕食者不参与群体协同行为
    for agent_id in runtime_state.prey_list:
        focal_agent = actors[agent_id]
        agent_idx = agent_id - 1  # 数组索引从0开始

        # get_topology_neighbors 现在接受 runtime_state 作为参数
        neighbors, _, neighbors_id_list = get_topology_neighbors(
            runtime_state, focal_agent.id
        )

        # 计算并记录邻居朝向熵
        if len(neighbors_id_list) > 0:
            neighbor_headings = [
                np.arctan2(actors[j].vel[1], actors[j].vel[0])
                for j in neighbors_id_list
            ]
            # metrics 字段通过 runtime_state.metrics 访问
            metrics.entropy_values[runtime_state.sim_step - 1, agent_idx] = (
                calculate_heading_entropy(neighbor_headings)
            )

        # 记录当前MST值
        metrics.mst_values[runtime_state.sim_step - 1, agent_idx] = (
            focal_agent.cj_threshold
        )

        # 初始化邻居观测字段，以处理没有邻居的情况
        focal_agent.observed_neighbor_ids = []
        focal_agent.observed_neighbor_cj_values = []
        focal_agent.observed_neighbor_relative_positions = []

        if neighbors:
            focal_agent.observed_neighbor_ids = neighbors_id_list

            # get_candidate_neighbors 现在接受 runtime_state 作为参数
            temp_neighbor_cj_values = get_candidate_neighbors(
                focal_agent.id, neighbors, runtime_state
            )
            focal_agent.observed_neighbor_cj_values = temp_neighbor_cj_values

            relative_positions = []
            for (
                neighbor_id_val
            ) in neighbors_id_list:  # 为了清晰，将 j 重命名为 neighbor_id_val
                relative_positions.append(
                    actors[neighbor_id_val].pose - focal_agent.pose
                )
            focal_agent.observed_neighbor_relative_positions = np.array(
                relative_positions
            )

            # 原有的激活逻辑继续
            if focal_agent.is_activated and focal_agent.src_id is not None:
                src_id = focal_agent.src_id
                # 检查 src_id 是否仍然是有效个体（例如，未被捕获）
                if src_id in actors:
                    src_agent = actors[src_id]
                    if (
                        np.linalg.norm(src_agent.vel - focal_agent.vel)
                        < config.deac_threshold  # 来自配置
                    ):
                        focal_agent.is_activated = False
                        focal_agent.src_id = None
                        temp_vel_sum = np.array([0.0, 0.0])
                        for neighbor_id_val_sum in neighbors_id_list:
                            temp_vel_sum += unitvel(actors[neighbor_id_val_sum].vel)
                        dir_vel_align[agent_idx] = unitvel(temp_vel_sum)
                    else:
                        dir_vel_align[agent_idx] = unitvel(src_agent.vel)
                else:  # 源个体可能已被移除（例如被捕获）
                    focal_agent.is_activated = False
                    focal_agent.src_id = None
                    # 回退到当前邻居的平均速度
                    temp_vel_sum = np.array([0.0, 0.0])
                    for neighbor_id_val_sum in neighbors_id_list:
                        temp_vel_sum += unitvel(actors[neighbor_id_val_sum].vel)
                    dir_vel_align[agent_idx] = unitvel(temp_vel_sum)

            else:  # 未激活或没有 src_id
                # 如果可用，则使用已计算的 cj 值，否则重新计算
                # 假设 temp_neighbor_cj_values 是当前邻居的 cj 值
                cj_values_for_activation = temp_neighbor_cj_values

                if (
                    len(cj_values_for_activation) > 0
                    and np.max(cj_values_for_activation) > focal_agent.cj_threshold
                ):
                    focal_agent.is_activated = True
                    metrics.maxcj[runtime_state.sim_step - 1] = np.max(
                        cj_values_for_activation
                    )
                    max_cj_index = np.argmax(cj_values_for_activation)
                    src_agent_candidate = neighbors[max_cj_index]
                    focal_agent.src_id = src_agent_candidate.id
                    dir_vel_align[agent_idx] = unitvel(src_agent_candidate.vel)
                else:
                    temp_vel_sum = np.array([0.0, 0.0])
                    for neighbor_id_val_sum in neighbors_id_list:
                        temp_vel_sum += unitvel(actors[neighbor_id_val_sum].vel)
                    dir_vel_align[agent_idx] = unitvel(temp_vel_sum)

            # 计算并记录邻居激活比例
            activated_neighbor_count = sum(
                1
                for neighbor_id_val_count in neighbors_id_list
                if actors[neighbor_id_val_count].is_activated
            )
            metrics.activation_ratios[runtime_state.sim_step - 1, agent_idx] = (
                activated_neighbor_count / len(neighbors_id_list)
                if neighbors_id_list
                else 0
            )
        else:  # 没有邻居
            # 如果没有邻居，个体继续依靠自驱动或默认行为
            # 对于对齐行为，如果没有邻居，它可能会保持当前的速度方向
            dir_vel_align[agent_idx] = unitvel(focal_agent.vel)

        if focal_agent.is_activated and focal_agent.src_id is not None:
            activated_this_step.append(agent_id)
            activated_src_this_step.append(focal_agent.src_id)

    # 保存本步骤的激活信息
    metrics.activated_ids.append(activated_this_step)
    metrics.activated_count[runtime_state.sim_step - 1] = len(activated_this_step)
    metrics.activated_src_ids.append(activated_src_this_step)

    # ----- AGGREGATION: 个体位置协同 -----
    dir_pos_coh_rep = np.zeros((num_agents_for_arrays, 2))
    # 只为猎物计算位置协同，捕食者不参与群体内聚/排斥行为
    for agent_id_coh in runtime_state.prey_list:
        agent_idx_coh = agent_id_coh - 1
        current_agent_pose = actors[agent_id_coh].pose

        # 考虑所有其他猎物的内聚/排斥作用（猎物之间的相互作用）
        # 注意：捕食者不参与猎物间的内聚/排斥计算
        temp_pos_force = np.array([0.0, 0.0])
        for other_agent_id in runtime_state.prey_list:
            if agent_id_coh == other_agent_id:
                continue

            other_agent_pose = actors[other_agent_id].pose
            rij = other_agent_pose - current_agent_pose
            dij = np.linalg.norm(rij)

            if dij == 0:  # 避免个体在完全相同位置时除以零
                nij = np.zeros(2)
                ra = 10000  # 如果距离为零，则没有力（或非常强的排斥力）
            else:
                nij = rij / dij
                if dij <= config.drep:  # 排斥
                    ra = config.weight_rep * (dij / config.drep - 1)
                elif dij <= config.dsen:  # 吸引（仅在 dsen 范围内）
                    # 原始公式: 1 - (dsen - dij) / (dsen - drep)
                    # 确保 dsen > drep 以避免除以零或负结果
                    denominator = config.dsen - config.drep
                    if denominator <= 0:  # 正常参数下不应发生
                        factor = 0
                    else:
                        factor = (
                            dij - config.drep
                        ) / denominator  # 吸引区内的归一化距离 [0,1]
                    ra = config.weight_att * factor
                else:  # 超出 dsen 则没有吸引力
                    ra = 0

            temp_pos_force += ra * nij
        dir_pos_coh_rep[agent_idx_coh] = unitvel(temp_pos_force)

    # ----- 远离attacker -----
    dir_esc_hawk = np.zeros((num_agents_for_arrays, 2))
    warn_ids_current_step = []

    if config.hawk_id in actors:  # 检查攻击者是否存在
        hawk_agent = actors[config.hawk_id]
        hawk_pos = hawk_agent.pose

        # 直接使用prey_list，无需额外判断
        for prey_id in runtime_state.prey_list:

            prey_agent = actors[prey_id]
            prey_idx = prey_id - 1

            vec2hawk = hawk_pos - prey_agent.pose
            dist2hawk = np.linalg.norm(vec2hawk)

            # config.r_escape 是一个数组，通过 prey_idx 索引
            if dist2hawk <= config.r_escape[prey_idx]:
                warn_ids_current_step.append(prey_id)
                if dist2hawk > 0:  # 避免攻击者与猎物位置重合时除以零
                    dir_esc_hawk[prey_idx] = -(vec2hawk / dist2hawk)  # 直接逃离
                else:  # 如果距离为零，选择随机逃逸方向或保持不动
                    dir_esc_hawk[prey_idx] = unitvel(np.random.rand(2) - 0.5)

    metrics.warn_ids.append(warn_ids_current_step)
    metrics.warn_num[runtime_state.sim_step - 1] = len(warn_ids_current_step)

    # ----- 行为综合：位置协同&速度协同&避险机动 -----
    # 以自驱动为基础
    # temp_social_dir = i_vel_prop # 原始逻辑中缺失此部分，添加自驱动
    # 已修正：结合内聚/排斥与对齐
    temp_social_dir = dir_pos_coh_rep + config.weight_align * dir_vel_align

    # 添加自驱动 (i_vel_prop 在开始时已计算)
    # 原始 MATLAB 代码可能在 des_turn_angle 的使用中隐式包含了自驱动。
    # 此处显式添加或确保其为最终 des_dir 的一部分。
    # 目前，假设内聚/排斥和对齐的组合是主要的社会力。
    # 自驱动则是关于在其他力不占主导时维持当前动量。
    # 一种常见方式：desired_direction = w_social * social_dir + w_self * self_dir
    # 假设 i_vel_prop 是基础，社会力对其进行修正。
    # 或者，下面计算的 des_dir 是“社会期望方向”，
    # 然后将其与当前的 i_vel_prop 进行比较以获得转向角。

    des_dir_combined = np.zeros_like(temp_social_dir)
    for i in range(num_agents_for_arrays):  # 迭代 0 到 max_id-1
        # 如果没有社会力，个体可能依赖其 i_vel_prop (自驱动)
        # 如果 temp_social_dir[i] 为零，此逻辑需要稳健。
        if np.all(temp_social_dir[i] == 0):
            des_dir_combined[i] = i_vel_prop[i]  # 回退到自驱动方向
        else:
            des_dir_combined[i] = unitvel(temp_social_dir[i])

    # 避险机动会覆盖受警告个体的其他行为
    if warn_ids_current_step:
        for warned_agent_id in warn_ids_current_step:
            warned_agent_idx = warned_agent_id - 1
            # 在缩放前确保逃逸方向非零
            # 原始逻辑是：des_dir = weight_esc * dir_esc (其中 dir_esc 是单位向量)
            # angle_of_vectors 无论如何都会对其输入进行归一化。
            # 因此，为了在 weight_esc != 1 时更接近原始行为：
            if np.any(dir_esc_hawk[warned_agent_idx]):  # 如果存在逃逸方向
                des_dir_combined[warned_agent_idx] = (
                    config.weight_esc * dir_esc_hawk[warned_agent_idx]
                )
            # 如果 dir_esc_hawk 为零，则保持为零。

    # 默认速度
    des_speed_final = config.v0 * np.ones(num_agents_for_arrays)

    # ----- attacker的运动过程 -----
    # 使用predator_list检查，更明确的语义
    if runtime_state.predator_list and config.hawk_id in runtime_state.predator_list:
        hawk_agent_idx = config.hawk_id - 1

        # 默认攻击者状态 (在 attack_step 之前或没有猎物时)
        # TODO: 攻击者的默认方向/速度应可配置
        des_dir_combined[hawk_agent_idx] = np.array([-1.0, 0.0])
        des_speed_final[hawk_agent_idx] = 0.0

        if runtime_state.sim_step >= config.attack_step:
            pos_hawk = actors[config.hawk_id].pose

            # dist_hawk2prey_all = np.full(
            #     num_agents_for_arrays, np.inf
            # )  # 对非猎物使用 np.inf
            # # dir_hawk2prey_all = np.zeros((num_agents_for_arrays, 2))

            # 直接使用prey_list，更清晰
            prey_ids = runtime_state.prey_list
            if prey_ids:  # 如果有任何猎物
                prey_poses = np.array([actors[pid].pose for pid in prey_ids])
                vecs_hawk_to_prey = prey_poses - pos_hawk
                dists_to_prey = np.linalg.norm(vecs_hawk_to_prey, axis=1)

                min_dist_idx = np.argmin(dists_to_prey)
                actual_target_dist = dists_to_prey[min_dist_idx]
                # target_prey_id = prey_ids[min_dist_idx]

                metrics.target_dist[runtime_state.sim_step - 1] = actual_target_dist

                # 朝向最近猎物的方向
                if actual_target_dist > 0:
                    des_dir_combined[hawk_agent_idx] = unitvel(
                        vecs_hawk_to_prey[min_dist_idx]
                    )
                else:  # 攻击者在猎物正上方，可以保持当前方向或默认方向
                    des_dir_combined[hawk_agent_idx] = unitvel(
                        actors[config.hawk_id].vel
                        if np.any(actors[config.hawk_id].vel)
                        else np.array([1.0, 0.0])
                    )

                des_speed_final[hawk_agent_idx] = config.v0_hawk
            else:  # 没有猎物了
                metrics.target_dist[runtime_state.sim_step - 1] = np.nan  # 没有目标

    # ----- 转换为t+1期望转动角度 -----
    des_turn_angle_final = np.zeros(num_agents_for_arrays)

    for agent_id_turn in runtime_state.robots_list:
        agent_idx_turn = agent_id_turn - 1
        current_agent_vel_dir = unitvel(actors[agent_id_turn].vel)

        # 确保 des_dir_combined 不是用于角度计算的零向量
        desired_dir_for_agent = des_dir_combined[agent_idx_turn]
        if not np.any(desired_dir_for_agent):  # 如果期望方向是零向量
            # 保持当前方向 (零转向角)
            des_turn_angle_final[agent_idx_turn] = 0.0
        else:
            des_turn_angle_final[agent_idx_turn] = angle_of_vectors(
                current_agent_vel_dir.reshape(2, 1), desired_dir_for_agent.reshape(2, 1)
            )[0]

        # 保存个体的期望运动状态
        actors[agent_id_turn].desired_turn_angle.append(
            des_turn_angle_final[agent_idx_turn]
        )
        actors[agent_id_turn].desired_speed.append(des_speed_final[agent_idx_turn])

    return des_turn_angle_final, des_speed_final, runtime_state
