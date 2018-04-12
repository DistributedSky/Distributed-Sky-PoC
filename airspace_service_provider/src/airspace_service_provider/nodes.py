from . import airspace_service_provider


def airspace_service_provider_node():
    airspace_service_provider.AirspaceServiceProvider().spin()
