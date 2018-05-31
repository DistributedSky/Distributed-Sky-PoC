from . import route_confirm


def route_confirmer_node():
    route_confirm.RouteConfirmer().spin()
