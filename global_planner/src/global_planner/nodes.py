from . import global_planner


def global_planner_node():
    global_planner.GlobalPlanner().spin()
