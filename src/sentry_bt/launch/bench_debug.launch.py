from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario_path = PathJoinSubstitution([
        FindPackageShare('sentry_bt'),
        LaunchConfiguration('scenario'),
    ])

    debug_topic = LaunchConfiguration('debug_topic')
    intent_topic = LaunchConfiguration('intent_topic')
    node_output = LaunchConfiguration('node_output')

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario',
            default_value='posture_stress.yaml',
            description='Installed scenario YAML filename'),
        DeclareLaunchArgument(
            'tick_hz',
            default_value='20.0',
            description='Decision and simulator tick rate'),
        DeclareLaunchArgument(
            'enable_bt_file_log',
            default_value='true',
            description='Whether sentry_bt writes BT log files'),
        DeclareLaunchArgument(
            'btlog_path',
            default_value='/tmp/sentry_bt.btlog',
            description='BehaviorTree log output path'),
        DeclareLaunchArgument(
            'enable_groot_zmq',
            default_value='false',
            description='Enable Groot live publisher'),
        DeclareLaunchArgument(
            'node_output',
            default_value='log',
            description='Output mode for sentry_bt and sentry_bt_sim: screen or log'),
        DeclareLaunchArgument(
            'debug_topic',
            default_value='/sentry_bt/debug',
            description='Structured debug topic name'),
        DeclareLaunchArgument(
            'intent_topic',
            default_value='/sentry/intent',
            description='Intent topic published by sentry_bt'),
        DeclareLaunchArgument(
            'show_debug_echo',
            default_value='false',
            description='Whether to auto-run ros2 topic echo on the debug topic'),
        DeclareLaunchArgument(
            'show_debug_watch',
            default_value='true',
            description='Whether to run the structured debug watch tool'),
        DeclareLaunchArgument(
            'watch_fields',
            default_value=(
                'frame_index,tactical_state,rule_action_type,'
                'reported_posture,current_posture,desired_posture,pending_posture_target,'
                'posture_switch_pending,posture_cooldown_remaining_ms,'
                'current_posture_remaining_before_debuff_ms,current_posture_debuffed,'
                'hp,ammo_17,heat,enemy_in_view,referee_link_fresh,sim_input_fresh,'
                'desired_goal,fire_policy,spin_mode'
            ),
            description='Comma-separated fields shown by sentry_bt_debug_watch'),
        DeclareLaunchArgument(
            'watch_only_changed',
            default_value='false',
            description='Whether sentry_bt_debug_watch only shows changed fields'),
        DeclareLaunchArgument(
            'watch_history_size',
            default_value='80',
            description='History length shown by sentry_bt_debug_watch'),
        DeclareLaunchArgument(
            'watch_refresh_hz',
            default_value='8.0',
            description='Refresh rate of sentry_bt_debug_watch'),
        DeclareLaunchArgument(
            'watch_view',
            default_value='full',
            description='Debug watch view: table, io, or full'),
        DeclareLaunchArgument(
            'posture_switch_cooldown_ms',
            default_value='5000',
            description='Decision-side posture switch cooldown'),
        DeclareLaunchArgument(
            'posture_feedback_stable_ms',
            default_value='200',
            description='Reported posture debounce window'),
        DeclareLaunchArgument(
            'posture_debuff_threshold_ms',
            default_value='180000',
            description='Accumulated posture time threshold before posture debuff applies'),
        DeclareLaunchArgument(
            'posture_debuff_rotate_margin_ms',
            default_value='15000',
            description='Early-rotation margin before posture debuff threshold'),
        DeclareLaunchArgument(
            'status_timeout_ms',
            default_value='300',
            description='Maximum accepted age for /gimbal_status before fail-safe'),
        DeclareLaunchArgument(
            'enemy_memory_ms',
            default_value='800',
            description='Short enemy-track memory used to absorb detector flicker'),
        DeclareLaunchArgument(
            'sim_posture_apply_delay_ms',
            default_value='150',
            description='Simulator delay before posture feedback changes'),
        DeclareLaunchArgument(
            'disengage_delay_ms',
            default_value='6000',
            description='Simulator disengage recovery delay'),

        Node(
            package='sentry_bt',
            executable='sentry_bt_sim',
            name='sentry_bt_sim',
            output=node_output,
            emulate_tty=True,
            parameters=[{
                'tick_hz': LaunchConfiguration('tick_hz'),
                'intent_topic': intent_topic,
                'scenario_path': scenario_path,
                'sim_posture_apply_delay_ms': LaunchConfiguration('sim_posture_apply_delay_ms'),
                'disengage_delay_ms': LaunchConfiguration('disengage_delay_ms'),
            }],
        ),

        Node(
            package='sentry_bt',
            executable='sentry_bt',
            name='sentry_bt',
            output=node_output,
            emulate_tty=True,
            parameters=[{
                'tick_hz': LaunchConfiguration('tick_hz'),
                'enable_sim_input': True,
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
            }],
        ),

        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('show_debug_echo')),
            cmd=['ros2', 'topic', 'echo', debug_topic],
            output='screen',
            emulate_tty=True,
        ),

        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('show_debug_watch')),
            cmd=[
                'ros2', 'run', 'sentry_bt', 'sentry_bt_debug_watch', '--',
                '--topic', debug_topic,
                '--fields', LaunchConfiguration('watch_fields'),
                '--only-changed', LaunchConfiguration('watch_only_changed'),
                '--history-size', LaunchConfiguration('watch_history_size'),
                '--refresh-hz', LaunchConfiguration('watch_refresh_hz'),
                '--sim-input-topic', '/sentry_bt/sim_input',
                '--intent-topic', intent_topic,
                '--status-topic', '/gimbal_status',
                '--mode', 'debug',
                '--view', LaunchConfiguration('watch_view'),
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
