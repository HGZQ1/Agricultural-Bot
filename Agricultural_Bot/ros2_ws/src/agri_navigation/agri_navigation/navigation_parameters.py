"""Configure navigation inputs while keeping localization inputs independent."""

from copy import deepcopy


def configure_navigation_parameters(
    parameters: dict,
    obstacle_scan: str,
    navigation_map_topic: str | None = None,
) -> dict:
    """Return Nav2 parameters with explicit obstacle and optional map inputs.

    Costmap nodes are children of the planner/controller servers, so their
    parameters must be changed in their YAML namespaces, rather than through
    an inline parameter dictionary addressed to the parent server.
    """
    if not isinstance(parameters, dict):
        raise ValueError('navigation parameters must be a YAML mapping')
    if not isinstance(obstacle_scan, str) or not obstacle_scan:
        raise ValueError('obstacle_scan must name a non-empty ROS topic')
    if navigation_map_topic is not None and (
            not isinstance(navigation_map_topic, str)
            or not navigation_map_topic):
        raise ValueError('navigation_map_topic must name a non-empty ROS topic')

    configured = deepcopy(parameters)
    for name in ('global_costmap', 'local_costmap'):
        try:
            costmap = configured[name][name]['ros__parameters']
            obstacle = costmap['obstacle_layer']['scan']
        except (KeyError, TypeError) as error:
            raise ValueError(
                f'navigation parameters require {name} obstacle_layer.scan') from error
        obstacle['topic'] = obstacle_scan
        if navigation_map_topic is None:
            continue
        plugins = costmap.get('plugins')
        if not isinstance(plugins, list):
            raise ValueError(f'{name}.plugins must be a YAML list')
        if 'static_layer' not in plugins:
            plugins.insert(0, 'static_layer')
        static = costmap.setdefault('static_layer', {})
        static.setdefault('plugin', 'nav2_costmap_2d::StaticLayer')
        static['enabled'] = True
        static['map_topic'] = navigation_map_topic
        static['map_subscribe_transient_local'] = True

    try:
        monitor = configured['collision_monitor']['ros__parameters']['scan']
    except (KeyError, TypeError) as error:
        raise ValueError(
            'navigation parameters require collision_monitor.scan') from error
    monitor['topic'] = obstacle_scan
    return configured
