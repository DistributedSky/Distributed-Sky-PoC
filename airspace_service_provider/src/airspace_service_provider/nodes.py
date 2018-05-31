from . import airspace_service_provider, registry_connector


def airspace_service_provider_node():
    airspace_service_provider.AirspaceServiceProvider().spin()


def registry_connector_node():
    registry_connector.RegistryConnector().spin()
