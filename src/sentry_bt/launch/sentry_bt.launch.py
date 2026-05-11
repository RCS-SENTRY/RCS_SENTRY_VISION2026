from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    is_debug = PythonExpression(["'", mode, "' == 'debug'"])
    is_onboard = PythonExpression(["'", mode, "' == 'onboard'"])

    scenario_path = PathJoinSubstitution([
        FindPackageShare('sentry_bt'),
        LaunchConfiguration('scenario'),
    ])

    debug_topic = LaunchConfiguration('debug_topic')
    intent_topic = LaunchConfiguration('intent_topic')
    status_topic = LaunchConfiguration('status_topic')
    sim_input_topic = LaunchConfiguration('sim_input_topic')
    node_output = LaunchConfiguration('node_output')

    common_bt_params = {
        'tick_hz': LaunchConfiguration('tick_hz'),
        'enable_bt_file_log': LaunchConfiguration('enable_bt_file_log'),
        'btlog_path': LaunchConfiguration('btlog_path'),
        'enable_groot_zmq': LaunchConfiguration('enable_groot_zmq'),
        'debug_topic': debug_topic,
        'intent_topic': intent_topic,
        'posture_switch_cooldown_ms': LaunchConfiguration('posture_switch_cooldown_ms'),
        'posture_feedback_stable_ms': LaunchConfiguration('posture_feedback_stable_ms'),
        'posture_debuff_threshold_ms': LaunchConfiguration('posture_debuff_threshold_ms'),
        'posture_debuff_rotate_margin_ms': LaunchConfiguration('posture_debuff_rotate_margin_ms'),
        'status_timeout_ms': LaunchConfiguration('status_timeout_ms'),
        'enemy_memory_ms': LaunchConfiguration('enemy_memory_ms'),
        'hp_resupply_enter_ratio': LaunchConfiguration('hp_resupply_enter_ratio'),
        'hp_resupply_exit_ratio': LaunchConfiguration('hp_resupply_exit_ratio'),
        'ammo_resupply_enter_count': LaunchConfiguration('ammo_resupply_enter_count'),
        'ammo_resupply_exit_count': LaunchConfiguration('ammo_resupply_exit_count'),
        'resupply_rfid_confirm_hold_ms': LaunchConfiguration('resupply_rfid_confirm_hold_ms'),
        'resupply_goal_timeout_ms': LaunchConfiguration('resupply_goal_timeout_ms'),
        'resupply_wait_recovery_timeout_ms': LaunchConfiguration('resupply_wait_recovery_timeout_ms'),
        'resupply_candidate_switch_cooldown_ms': LaunchConfiguration('resupply_candidate_switch_cooldown_ms'),
        'resupply_candidates': ['SUPPLY_LEFT', 'SUPPLY_RIGHT', 'BASE_HOME'],
        'dead_return_home_enabled': LaunchConfiguration('dead_return_home_enabled'),
        'dead_return_goal': LaunchConfiguration('dead_return_goal'),
        'dead_return_rfid_confirm_hold_ms': LaunchConfiguration('dead_return_rfid_confirm_hold_ms'),
        'dead_full_hp_exit_ratio': LaunchConfiguration('dead_full_hp_exit_ratio'),
        'dead_return_no_timeout': LaunchConfiguration('dead_return_no_timeout'),
        'goal_dwell_default_ms': LaunchConfiguration('goal_dwell_default_ms'),
        'goal_dwell_search_ms': LaunchConfiguration('goal_dwell_search_ms'),
        'goal_dwell_hold_ms': LaunchConfiguration('goal_dwell_hold_ms'),
        'goal_dwell_resupply_ms': LaunchConfiguration('goal_dwell_resupply_ms'),
        'goal_dwell_engage_ms': LaunchConfiguration('goal_dwell_engage_ms'),
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='debug',
            description='debug starts sentry_bt_sim with selectable scenario; onboard uses real topics only'),
        DeclareLaunchArgument(
            'scenario',
            default_value='nominal_patrol.yaml',
            description=(
                'Debug scenario file: nominal_patrol.yaml, engage_resupply.yaml, '
                'critical_hp_remote_repair.yaml, revive_cycle.yaml, posture_stress.yaml, '
                'energy_activation.yaml, input_loss_failsafe.yaml, rfid_field_points.yaml'
            )),
        DeclareLaunchArgument('tick_hz', default_value='20.0'),
        DeclareLaunchArgument('node_output', default_value='screen'),
        DeclareLaunchArgument('enable_bt_file_log', default_value='true'),
        DeclareLaunchArgument('btlog_path', default_value='/tmp/sentry_bt.btlog'),
        DeclareLaunchArgument('enable_groot_zmq', default_value='false'),
        DeclareLaunchArgument('debug_topic', default_value='/sentry_bt/debug'),
        DeclareLaunchArgument('intent_topic', default_value='/sentry/intent'),
        DeclareLaunchArgument('status_topic', default_value='/gimbal_status'),
        DeclareLaunchArgument('sim_input_topic', default_value='/sentry_bt/sim_input'),
        DeclareLaunchArgument(
            'enable_sim_input',
            default_value='true',
            description='Debug-mode decision input source. Set false to test RFID/event-data status paths.'),
        DeclareLaunchArgument('show_debug_watch', default_value='true'),
        DeclareLaunchArgument('watch_view', default_value='io'),
        DeclareLaunchArgument(
            'watch_fields',
            default_value=(
                'frame_index,tactical_state,rule_action_type,desired_goal,'
                'reported_posture,current_posture,desired_posture,fire_policy,spin_mode,'
                'hp,ammo_17,heat,enemy_in_view,referee_link_fresh,sim_input_fresh,'
                'nav_goal_active,nav_goal_reached,current_goal_id,'
                'autoaim_has_target,autoaim_tracking,autoaim_fire_ready,'
                'posture_cmd_referee,remote_ammo_req_inc,remote_hp_req_inc,'
                'activate_energy_confirm,claim_periodic_ammo'
            )),
        DeclareLaunchArgument('watch_only_changed', default_value='false'),
        DeclareLaunchArgument('watch_history_size', default_value='80'),
        DeclareLaunchArgument('watch_refresh_hz', default_value='6.0'),
        DeclareLaunchArgument('posture_switch_cooldown_ms', default_value='5000'),
        DeclareLaunchArgument('posture_feedback_stable_ms', default_value='200'),
        DeclareLaunchArgument('posture_debuff_threshold_ms', default_value='180000'),
        DeclareLaunchArgument('posture_debuff_rotate_margin_ms', default_value='15000'),
        DeclareLaunchArgument('status_timeout_ms', default_value='300'),
        DeclareLaunchArgument('enemy_memory_ms', default_value='800'),
        DeclareLaunchArgument('hp_resupply_enter_ratio', default_value='0.35'),
        DeclareLaunchArgument('hp_resupply_exit_ratio', default_value='0.60'),
        DeclareLaunchArgument('ammo_resupply_enter_count', default_value='80'),
        DeclareLaunchArgument('ammo_resupply_exit_count', default_value='120'),
        DeclareLaunchArgument('resupply_rfid_confirm_hold_ms', default_value='300'),
        DeclareLaunchArgument('resupply_goal_timeout_ms', default_value='12000'),
        DeclareLaunchArgument('resupply_wait_recovery_timeout_ms', default_value='12000'),
        DeclareLaunchArgument('resupply_candidate_switch_cooldown_ms', default_value='1500'),
        DeclareLaunchArgument('resupply_candidates', default_value='["SUPPLY_LEFT", "SUPPLY_RIGHT", "BASE_HOME"]'),
        DeclareLaunchArgument('dead_return_home_enabled', default_value='true'),
        DeclareLaunchArgument('dead_return_goal', default_value='BASE_HOME'),
        DeclareLaunchArgument('dead_return_rfid_confirm_hold_ms', default_value='300'),
        DeclareLaunchArgument('dead_full_hp_exit_ratio', default_value='0.98'),
        DeclareLaunchArgument('dead_return_no_timeout', default_value='true'),
        DeclareLaunchArgument('goal_dwell_default_ms', default_value='1500'),
        DeclareLaunchArgument('goal_dwell_search_ms', default_value='2500'),
        DeclareLaunchArgument('goal_dwell_hold_ms', default_value='3000'),
        DeclareLaunchArgument('goal_dwell_resupply_ms', default_value='0'),
        DeclareLaunchArgument('goal_dwell_engage_ms', default_value='0'),
        DeclareLaunchArgument('sim_posture_apply_delay_ms', default_value='150'),
        DeclareLaunchArgument('disengage_delay_ms', default_value='6000'),

        Node(
            condition=IfCondition(is_debug),
            package='sentry_bt',
            executable='sentry_bt_sim',
            name='sentry_bt_sim',
            output=node_output,
            emulate_tty=True,
            parameters=[{
                'tick_hz': LaunchConfiguration('tick_hz'),
                'intent_topic': intent_topic,
                'status_topic': status_topic,
                'sim_input_topic': sim_input_topic,
                'scenario_path': scenario_path,
                'sim_posture_apply_delay_ms': LaunchConfiguration('sim_posture_apply_delay_ms'),
                'disengage_delay_ms': LaunchConfiguration('disengage_delay_ms'),
            }],
        ),

        Node(
            condition=IfCondition(is_debug),
            package='sentry_bt',
            executable='sentry_bt',
            name='sentry_bt',
            output=node_output,
            emulate_tty=True,
            parameters=[{
                **common_bt_params,
                'enable_sim_input': LaunchConfiguration('enable_sim_input'),
            }],
        ),

        Node(
            condition=IfCondition(is_onboard),
            package='sentry_bt',
            executable='sentry_bt',
            name='sentry_bt',
            output=node_output,
            emulate_tty=True,
            parameters=[{
                **common_bt_params,
                'enable_sim_input': False,
            }],
        ),

        ExecuteProcess(
            condition=IfCondition(PythonExpression([
                "'", mode, "' == 'debug' and '", LaunchConfiguration('show_debug_watch'), "' == 'true'"
            ])),
            cmd=[
                'ros2', 'run', 'sentry_bt', 'sentry_bt_debug_watch', '--',
                '--topic', debug_topic,
                '--fields', LaunchConfiguration('watch_fields'),
                '--only-changed', LaunchConfiguration('watch_only_changed'),
                '--history-size', LaunchConfiguration('watch_history_size'),
                '--refresh-hz', LaunchConfiguration('watch_refresh_hz'),
                '--sim-input-topic', sim_input_topic,
                '--intent-topic', intent_topic,
                '--status-topic', status_topic,
                '--mode', 'debug',
                '--view', LaunchConfiguration('watch_view'),
            ],
            output='screen',
            emulate_tty=True,
        ),

        ExecuteProcess(
            condition=IfCondition(PythonExpression([
                "'", mode, "' == 'onboard' and '", LaunchConfiguration('show_debug_watch'), "' == 'true'"
            ])),
            cmd=[
                'ros2', 'run', 'sentry_bt', 'sentry_bt_debug_watch', '--',
                '--topic', debug_topic,
                '--fields', LaunchConfiguration('watch_fields'),
                '--only-changed', LaunchConfiguration('watch_only_changed'),
                '--history-size', LaunchConfiguration('watch_history_size'),
                '--refresh-hz', LaunchConfiguration('watch_refresh_hz'),
                '--sim-input-topic', sim_input_topic,
                '--intent-topic', intent_topic,
                '--status-topic', status_topic,
                '--mode', 'onboard',
                '--view', LaunchConfiguration('watch_view'),
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
